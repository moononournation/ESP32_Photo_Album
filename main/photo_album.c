/* ESP32 Photo Album

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include <dirent.h>

#include <time.h>
#include <errno.h>
#include <sys/fcntl.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "tftspi.h"
#include "tft.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "esp_attr.h"
#include <sys/time.h>
#include <unistd.h>
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "apps/sntp/sntp.h"
#include "nvs_flash.h"

// ==========================================================
// Define which spi bus to use TFT_VSPI_HOST or TFT_HSPI_HOST
#define SPI_BUS TFT_VSPI_HOST
// ==========================================================

#define SPIFFS_BASE_PATH "/spiffs"
#define TIMEZONE 8 // hour offset
#define TIMEADJ 20 // seconds advanced real time for prefetch and load image

#define MARGIN_X 12
#define MARGIN_Y 12

#define NTP_WAIT 20 // seconds to wait NTP return
#define NTP_RETRY 3

#define WAKE_PIN 12

#define FILENAME_SIZE 20
#define PHOTO_LIST_SIZE 150
#define SHOW_PHOTO_COUNT 5

static const char tag[] = "[Photo Album]";

/* FreeRTOS event group to signal when we are connected & ready to make a request */
static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
const int CONNECTED_BIT = 0x00000001;
const int DOWNLOADED_BIT = 0x00000010;

// ==========================================================
// The web server that getting the time picture,
// this is one of the famous site, you may search
// more on the web.
#define WEB_SERVER "10.0.1.6"
#define WEB_PORT "3200"
static const char *PHOTO_LIST_REQUEST =
		"GET / HTTP/1.0\r\n"
		"Host: " WEB_SERVER "\r\n"
		"User-Agent: esp-idf/1.0 esp32\r\n"
		"\r\n";
static const char *REQUEST_FORMAT =
		// "GET http://" WEB_SERVER "/assets/toppict/jp/t1/%.2d%.2d.jpg HTTP/1.0\r\n"
		// "GET http://" WEB_SERVER "/assets/pict/jp/pc/%.2d%.2d.jpg HTTP/1.0\r\n"
		// "GET http://" WEB_SERVER "/assets/pict/hiroshima/pc/%.2d%.2d.jpg HTTP/1.0\r\n"
		// "GET http://" WEB_SERVER ":" WEB_PORT "/%.2d.jpg HTTP/1.0\r\n"
		"GET /%s HTTP/1.0\r\n"
		"Host: " WEB_SERVER "\r\n"
		"User-Agent: esp-idf/1.0 esp32\r\n"
		"\r\n";
// ==========================================================

//------------------------------------------------------------
static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch (event->event_id)
	{
	case SYSTEM_EVENT_STA_START:
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		/* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

//-------------------------------
static void initialise_wifi(void)
{
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));
	ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
	wifi_config_t wifi_config = {
			.sta = {
					.ssid = "Moon On AirPort",
					.password = "MoonOnAP",
			},
	};

	ESP_LOGI(tag, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
}

static void enter_sleep()
{
	ESP_LOGI(tag, "Enter sleep...");

	gpio_set_level(PIN_NUM_BCKL, PIN_BCKL_OFF);
	esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
	esp_sleep_enable_ext0_wakeup(WAKE_PIN, 0);
	rtc_gpio_pulldown_dis(WAKE_PIN);
	rtc_gpio_pullup_en(WAKE_PIN);
	esp_deep_sleep_start();
}

static int http_request(const void *req_buf, int str_len)
{
	const struct addrinfo hints = {
			.ai_family = AF_INET,
			.ai_socktype = SOCK_STREAM,
	};
	struct addrinfo *res;
	struct in_addr *addr;
	int s;

	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT,
											false, true, portMAX_DELAY);

	int err = getaddrinfo(WEB_SERVER, WEB_PORT, &hints, &res);

	if (err != 0 || res == NULL)
	{
		ESP_LOGE(tag, "DNS lookup failed err=%d res=%p", err, res);
	}

	/* Code to print the resolved IP.
           Note: inet_ntoa is non-reentrant, look at ipaddr_ntoa_r for "real" code */
	addr = &((struct sockaddr_in *)res->ai_addr)->sin_addr;
	ESP_LOGI(tag, "DNS lookup succeeded. IP=%s", inet_ntoa(*addr));

	s = socket(res->ai_family, res->ai_socktype, 0);
	if (s < 0)
	{
		ESP_LOGE(tag, "... Failed to allocate socket.");
	}
	ESP_LOGI(tag, "... allocated socket");

	if (connect(s, res->ai_addr, res->ai_addrlen) != 0)
	{
		ESP_LOGE(tag, "... socket connect failed errno=%d", errno);
	}

	ESP_LOGI(tag, "... connected");
	freeaddrinfo(res);

	if (write(s, req_buf, str_len) < 0)
	{
		ESP_LOGE(tag, "... socket send failed");
	}
	ESP_LOGI(tag, "... socket send success");

	struct timeval receiving_timeout;
	receiving_timeout.tv_sec = 5;
	receiving_timeout.tv_usec = 0;
	if (setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &receiving_timeout,
								 sizeof(receiving_timeout)) < 0)
	{
		ESP_LOGE(tag, "... failed to set socket receiving timeout");
	}
	ESP_LOGI(tag, "... set socket receiving timeout success");

	return s; // return socket
}

static void download_photo()
{
	char photo_list[PHOTO_LIST_SIZE][FILENAME_SIZE];
	int r;
	char req_buf[128];
	char recv_buf[512]; // assume response header must smaller than buffer size
	int str_len = sprintf(req_buf, PHOTO_LIST_REQUEST);
	ESP_LOGI(tag, "%s", req_buf);

	int s = http_request(req_buf, str_len);
	bool found_offset = false;
	int curr_list_idx = 0;
	int curr_filename_idx = 0;
	r = read(s, recv_buf, sizeof(recv_buf));
	for (int i = 0; i < r; i++)
	{
		putchar(recv_buf[i]); // output header for debug only
		if (!found_offset)
		{
			// search response header end, double carriage return
			if ((recv_buf[i] == '\r') && (recv_buf[i + 1] == '\n') && (recv_buf[i + 2] == '\r') && (recv_buf[i + 3] == '\n'))
			{
				found_offset = true;
				i += 3;
			}
		}
		else
		{ // found_offset
			if (recv_buf[i] == ',')
			{
				photo_list[curr_list_idx][curr_filename_idx] = 0;
				curr_list_idx++;
				curr_filename_idx = 0;
			}
			else
			{
				photo_list[curr_list_idx][curr_filename_idx++] = recv_buf[i];
			}
		}
	}

	do
	{
		r = read(s, recv_buf, sizeof(recv_buf));
		for (int i = 0; i < r; i++)
		{
			if (recv_buf[i] == ',')
			{
				photo_list[curr_list_idx][curr_filename_idx] = 0;
				curr_list_idx++;
				curr_filename_idx = 0;
			}
			else
			{
				photo_list[curr_list_idx][curr_filename_idx++] = recv_buf[i];
			}
		}
	} while (r > 0);
	close(s);
	photo_list[curr_list_idx][curr_filename_idx] = 0;

	char filename_buf[FILENAME_SIZE + sizeof(SPIFFS_BASE_PATH)];
	if (curr_list_idx > 0)
	{
		DIR *dp;
		struct dirent *ep;
		dp = opendir(SPIFFS_BASE_PATH "/");

		if (dp != NULL)
		{
			while ((ep = readdir(dp)))
			{
				bool found_photo = false;
				for (int i = 0; i <= curr_list_idx; i++)
				{
					if (strcmp(photo_list[i], ep->d_name) == 0)
					{
						found_photo = true;
					}
				}
				if (!found_photo)
				{
					sprintf(filename_buf, SPIFFS_BASE_PATH "/%s", ep->d_name);
					ESP_LOGI(tag, "Remove file: %s...", filename_buf);
					unlink(filename_buf);
				}
			}
		}
	}

	for (int i = 0; i <= curr_list_idx; i++)
	{
		sprintf(filename_buf, SPIFFS_BASE_PATH "/%s", photo_list[i]);

		struct stat st;
		if (stat(filename_buf, &st) != 0)
		{
			str_len = sprintf(req_buf, REQUEST_FORMAT, photo_list[i]);
			ESP_LOGI(tag, "%s", req_buf);

			s = http_request(req_buf, str_len);

			/* Read HTTP response */
			FILE *f = fopen(filename_buf, "w");
			if (f == NULL)
			{
				ESP_LOGE(tag, "Failed to open file for writing");
			}
			else
			{
				r = read(s, recv_buf, sizeof(recv_buf));
				int offset = -1;
				for (int i = 0; i < r; i++)
				{
					putchar(recv_buf[i]); // output header for debug only
					// search response header end, double carriage return
					if ((recv_buf[i] == '\r') && (recv_buf[i + 1] == '\n') && (recv_buf[i + 2] == '\r') && (recv_buf[i + 3] == '\n'))
					{
						offset = i + 4;
					}
				}
				int file_size = 0;
				// if found some response content at first buffer, start write from offset, otherwise start write from second buffer
				if ((offset > 0) && (offset < r))
				{
					ESP_LOGI(tag, "Found response content offset: %d", offset);
					// write response content (jpg file)
					file_size += fwrite(recv_buf + offset, 1, r - offset, f);
				}

				do
				{
					r = read(s, recv_buf, sizeof(recv_buf));
					file_size += fwrite(recv_buf, 1, r, f);
				} while (r > 0);
				fclose(f);
				ESP_LOGI(tag, "File written: %d", file_size);
			}
			close(s);
			ESP_LOGI(tag, "freemem=%d", esp_get_free_heap_size()); // show free heap for debug only
		}
	}

	esp_wifi_stop();
	xEventGroupSetBits(wifi_event_group, DOWNLOADED_BIT);
}

static void display_photo_task()
{
	DIR *dp;
	struct dirent *ep;
	dp = opendir(SPIFFS_BASE_PATH "/");
	int photo_count = 0;

	if (dp != NULL)
	{
		while ((ep = readdir(dp)))
		{
			ESP_LOGI(tag, "SPIFFS file %d: %s...", photo_count, ep->d_name);
			photo_count++;
		}
	}

	if (photo_count)
	{
		char filename_buf[FILENAME_SIZE + sizeof(SPIFFS_BASE_PATH)];
		for (int i = 0; i < SHOW_PHOTO_COUNT; i++)
		{
			seekdir(dp, (esp_random() % photo_count));
			ep = readdir(dp);
			sprintf(filename_buf, SPIFFS_BASE_PATH "/%s", ep->d_name);
			ESP_LOGI(tag, "Display %s...", filename_buf);

			TFT_jpg_image(CENTER, CENTER, 0, -1, filename_buf, NULL, 0);

			vTaskDelay(5000 / portTICK_PERIOD_MS);
		}
	}

	xEventGroupWaitBits(wifi_event_group, DOWNLOADED_BIT,
											false, true, portMAX_DELAY);
	enter_sleep();
}

//=============
void app_main()
{
	// ========  PREPARE DISPLAY INITIALIZATION  =========

	// === SET GLOBAL VARIABLES ==========================

	// ===================================================
	// ==== Set display type                         =====
	tft_disp_type = DEFAULT_DISP_TYPE;
	//tft_disp_type = DISP_TYPE_ILI9341;
	//tft_disp_type = DISP_TYPE_ILI9488;
	//tft_disp_type = DISP_TYPE_ST7735B;
	// ===================================================

	// ===================================================
	// === Set display resolution if NOT using default ===
	// === DEFAULT_TFT_DISPLAY_WIDTH &                 ===
	// === DEFAULT_TFT_DISPLAY_HEIGHT                  ===
	_width = DEFAULT_TFT_DISPLAY_WIDTH;		// smaller dimension
	_height = DEFAULT_TFT_DISPLAY_HEIGHT; // larger dimension
	//_width = 128;  // smaller dimension
	//_height = 160; // larger dimension
	// ===================================================

	// ===================================================
	// ==== Set maximum spi clock for display read    ====
	//      operations, function 'find_rd_speed()'    ====
	//      can be used after display initialization  ====
	max_rdclock = 8000000;
	// ===================================================

	// ====================================================================
	// === Pins MUST be initialized before SPI interface initialization ===
	// ====================================================================
	TFT_PinsInit();

	// ====  CONFIGURE SPI DEVICES(s)  ====================================================================================

	spi_lobo_device_handle_t spi;

	spi_lobo_bus_config_t buscfg = {
			.miso_io_num = PIN_NUM_MISO, // set SPI MISO pin
			.mosi_io_num = PIN_NUM_MOSI, // set SPI MOSI pin
			.sclk_io_num = PIN_NUM_CLK,	// set SPI CLK pin
			.quadwp_io_num = -1,
			.quadhd_io_num = -1,
			.max_transfer_sz = 6 * 1024,
	};
	spi_lobo_device_interface_config_t devcfg = {
			.clock_speed_hz = 8000000,				 // Initial clock out at 8 MHz
			.mode = 0,												 // SPI mode 0
			.spics_io_num = PIN_NUM_CS,				 // set SPI CS pin
			.flags = LB_SPI_DEVICE_HALFDUPLEX, // ALWAYS SET  to HALF DUPLEX MODE!! for display spi
	};

	// ====================================================================================================================

	vTaskDelay(500 / portTICK_RATE_MS);
	printf("\r\n==============================\r\n");
	printf("Pins used: miso=%d, mosi=%d, sck=%d, cs=%d\r\n", PIN_NUM_MISO, PIN_NUM_MOSI, PIN_NUM_CLK, PIN_NUM_CS);
	printf("==============================\r\n\r\n");

	// ==================================================================
	// ==== Initialize the SPI bus and attach the LCD to the SPI bus ====

	ESP_ERROR_CHECK(spi_lobo_bus_add_device(SPI_BUS, &buscfg, &devcfg, &spi));
	printf("SPI: display device added to spi bus (%d)\r\n", SPI_BUS);
	disp_spi = spi;

	// ==== Test select/deselect ====
	ESP_ERROR_CHECK(spi_lobo_device_select(spi, 1));
	ESP_ERROR_CHECK(spi_lobo_device_deselect(spi));

	printf("SPI: attached display device, speed=%u\r\n", spi_lobo_get_speed(spi));
	printf("SPI: bus uses native pins: %s\r\n", spi_lobo_uses_native_pins(spi) ? "true" : "false");

	// ================================
	// ==== Initialize the Display ====

	printf("SPI: display init...\r\n");
	TFT_display_init();
	printf("OK\r\n");

	// ---- Detect maximum read speed ----
	max_rdclock = find_rd_speed();
	printf("SPI: Max rd speed = %u\r\n", max_rdclock);

	// ==== Set SPI clock used for display operations ====
	spi_lobo_set_speed(spi, DEFAULT_SPI_CLOCK);
	printf("SPI: Changed speed to %u\r\n", spi_lobo_get_speed(spi));

	font_rotate = 0;
	text_wrap = 0;
	font_transparent = 0;
	font_forceFixed = 0;
	image_debug = 0;

	TFT_setGammaCurve(DEFAULT_GAMMA_CURVE);
	TFT_setRotation(LANDSCAPE);
	TFT_setFont(DEFAULT_FONT, NULL);

	ESP_ERROR_CHECK(nvs_flash_init());

	_fg = TFT_ORANGE;
	TFT_print("ESP32 Photo Album", MARGIN_X, MARGIN_Y);

	_fg = TFT_YELLOW;
	TFT_print("Initializing SPIFFS", MARGIN_X, LASTY + TFT_getfontheight() + 2);
	// ==== Initialize the file system ====
	esp_vfs_spiffs_conf_t conf = {
			.base_path = SPIFFS_BASE_PATH,
			.partition_label = NULL,
			.max_files = 5,
			.format_if_mount_failed = true};

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is an all-in-one convenience function.
	ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

	_fg = TFT_GREEN;
	TFT_print("Start DISPLAY PHOTO TASK...", MARGIN_X, LASTY + TFT_getfontheight() + 2);
	xTaskCreate(&display_photo_task, "display_photo_task", 4096, NULL, 4, NULL);

	_fg = TFT_BLUE;
	TFT_print("Initializing WiFi...", MARGIN_X, LASTY + TFT_getfontheight() + 2);
	initialise_wifi();

	_fg = TFT_MAGENTA;
	TFT_print("Download Photo from the Web...", MARGIN_X, LASTY + TFT_getfontheight() + 2);
	download_photo();
}
