// Arduino build process info: https://github.com/arduino/Arduino/wiki/Build-Process

#define WEBOTA_VERSION "0.1.5"

#include "WebOTA.h"
#include <Arduino.h>
#include <Update.h>
#include "./../../../include/log.h"

static const char *TAG = "WEBOTA";

WebOTA webota;

////////////////////////////////////////////////////////////////////////////

int WebOTA::init(WebServer *server, const char *path) {
	// Only run this once
	if(this->init_has_run){return 0;}

	add_http_routes(server, path);

	// Store that init has already run
	this->init_has_run = true;

	return 1;
}

long WebOTA::max_sketch_size() {
	long ret = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

	return ret;
}

// R Macro string literal https://en.cppreference.com/w/cpp/language/string_literal
const char ota_version_html[] PROGMEM = "<h1>Web-Update</h1>";

const char ota_upload_form[] PROGMEM = R"!^!(
<form method="POST" action="#" enctype="multipart/form-data" id="upload_form">
    <input type="file" name="update" id="file">
    <input type="submit" value="Update">
</form>

<div id="prg_wrap" style="border: 0px solid; width: 100%;">
   <div id="prg" style="text-shadow: 2px 2px 3px black; padding: 5px 0; display: none; border: 1px solid #008aff; background: #002180; text-align: center; color: white;"></div>
</div>

<script>
var domReady = function(callback) {
	document.readyState === "interactive" || document.readyState === "complete" ? callback() : document.addEventListener("DOMContentLoaded", callback);
};

domReady(function() {
	var myform = document.getElementById('upload_form');
	var filez  = document.getElementById('file');

	myform.onsubmit = function(event) {
		event.preventDefault();

		var formData = new FormData();
		var file     = filez.files[0];

		if (!file) { return false; }

		formData.append("files", file, file.name);

		var xhr = new XMLHttpRequest();
		xhr.upload.addEventListener("progress", function(evt) {
			if (evt.lengthComputable) {
				var per = Math.round((evt.loaded / evt.total) * 100);
				var prg = document.getElementById('prg');

				prg.innerHTML     = per + "%"
				prg.style.width   = per + "%"
				prg.style.display = "block"
			}
		}, false);
		xhr.open('POST', location.href, true);

		// Set up a handler for when the request finishes.
		xhr.onload = function () {
			if (xhr.status === 200) {
				//alert('Success');
			} else {
				//alert('An error occurred!');
			}
		};

		xhr.send(formData);
   }
});
</script>)!^!";



int WebOTA::add_http_routes(WebServer *server, const char *path) {
	// Upload firmware page
	server->on(path, HTTP_GET, [server,this]() {
		String html = FPSTR(ota_version_html);
		html += FPSTR(ota_upload_form);
		server->send_P(200, "text/html", html.c_str());
	});

	// Handling uploading firmware file
	server->on(path, HTTP_POST, [server,this]() {
		server->send(200, "text/plain", (Update.hasError()) ? "Update: fail\n" : "Update: OK!\n");
		delay(server, 500);
		ESP.restart();
	}, [server,this]() {
		HTTPUpload& upload = server->upload();

		if (upload.status == UPLOAD_FILE_START) {
			ESP_LOGI(TAG,"Firmware update initiated: %s",upload.filename.c_str());

			//uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
			uint32_t maxSketchSpace = this->max_sketch_size();

			if (!Update.begin(maxSketchSpace)) { //start with max available size
				ESP_LOGI(TAG,"Firmware update:", Update.errorString());
			}
		} else if (upload.status == UPLOAD_FILE_WRITE) {
			/* flashing firmware to ESP*/
			if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
				ESP_LOGI(TAG,"Firmware update:", Update.errorString());
			}

			// Store the next milestone to output
			uint16_t chunk_size  = 51200;
			static uint32_t next = 51200;

			// Check if we need to output a milestone (100k 200k 300k)
			if (upload.totalSize >= next) {
				ESP_LOGI(TAG,"%d k ",next/1024);
				next += chunk_size;
			}
		} else if (upload.status == UPLOAD_FILE_END) {
			if (Update.end(true)) { //true to set the size to the current progress
				ESP_LOGI(TAG,"Firmware update successful: %u bytes;\nRebooting...", upload.totalSize);
			} else {
				ESP_LOGI(TAG,"Firmware update:", Update.errorString());
			}
		}
	});

	return 1;
}

// If the MCU is in a delay() it cannot respond to HTTP OTA requests
// We do a "fake" looping delay and listen for incoming HTTP requests while waiting
void WebOTA::delay(WebServer *server, unsigned int ms) {
	// Borrowed from mshoe007 @ https://github.com/scottchiefbaker/ESP-WebOTA/issues/8
	decltype(millis()) last = millis();

	while ((millis() - last) < ms) {
		server->handleClient();
		::delay(5);
	}
}
