// Arduino build process info: https://github.com/arduino/Arduino/wiki/Build-Process

#define WEBOTA_VERSION "0.1.5"

#include "WebOTA.h"
#include <Arduino.h>
#include <Update.h>
#include "./../../../include/log.h"

static const char *TAG = "WEBOTA";

WebOTA webota;
WebServer *serverUpdate;


int WebOTA::init(WebServer *server, const char *path)
{
	// Only run this once
	if(this->init_has_run){return 0;}

	serverUpdate=server;
	add_http_routes(server, path);

	// Store that init has already run
	this->init_has_run = true;

	return 1;
}

long WebOTA::max_sketch_size()
{
	long ret = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;

	return ret;
}

// R Macro string literal https://en.cppreference.com/w/cpp/language/string_literal
const char ota_upload_form[] PROGMEM = R"!^!(
<!DOCTYPE HTML>
<html lang='de'>
<head>
<meta http-equiv='Content-Type' content='text/html' charset='utf-8'/>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>BSC</title>
<style>
  html {font-family:Helvetica; display:inline-block; margin:0px auto; text-align:left;}
  body {margin: 0; background-color: %s;}
  .content {padding:20px; max-width:600px;}
  .topnav {overflow: hidden;position:sticky;top:0;background-color:#6E6E6E;color:white;padding:5px;}
  .topnav span {float: left; padding: 14px 16px; text-decoration: none; font-size:1.7rem;}
  .btnBack:hover {background-color:#555555;color:white;}
  .hl {flex:1;font-size:2rem;}
  .titel {font-weight:bold; text-align:left; padding:5px;}

  progress
  {
	border: none;
	background-size: auto;
	width: 315px;
	height: 20px;
  }
</style>

<body>
  <div class="topnav">
    <span class='btnBack' onclick=location.href='../'>&#10094;</span>
    <span class='hl'>Web-Update</span>
  </div>
  <div class="content">
    <form method="POST" action="#" enctype="multipart/form-data" id="upload_form">
      <input type="file" name="update" id="file">
      <input type="submit" value="Update">
    </form>
    <br>
    <progress id='progress' max='100' value="0" ></progress>
    <br><br>
    <div id="status"></div>
  </div>

<script>
  const progress = document.getElementById('progress');
  var updateProgress=0;

  var domReady = function(callback)
  {
	document.readyState === "interactive" || document.readyState === "complete" ? callback() : document.addEventListener("DOMContentLoaded", callback);
  };

  domReady(function()
  {
    var myform = document.getElementById('upload_form');
    var filez  = document.getElementById('file');

    myform.onsubmit = function(event)
	{
	  event.preventDefault();
	  var formData = new FormData();
	  var file     = filez.files[0];
      if (!file) { return false; }

  	  getUpdateProgress();
	  document.getElementById('status').innerHTML='<b>Update l&auml;uft...</b>';
	  formData.append("files", file, file.name);

	  var xhr = new XMLHttpRequest();
	  xhr.upload.addEventListener("progress", function(evt)
	  {
		if (evt.lengthComputable) {
			var per = Math.round((evt.loaded / evt.total) * 100);
			if(per<100){updateProgress=Math.round(per/2);}
		}
	  }, false);
	  xhr.open('POST', location.href, true);

	  // Set up a handler for when the request finishes.
	  xhr.onload = function ()
	  {
		if (xhr.status === 200)
		{
		  //alert('Success');
		  updateProgress=100;
		  progress.value = updateProgress;
		  document.getElementById('status').innerHTML='<b>Update fertig! BSC wird neu gestartet...</b>';
		}else{
		  //alert('An error occurred!');
		  document.getElementById('status').innerHTML='<b>An error occurred!</b>';
		}
	  };
	  xhr.send(formData);
   }
  });

  function getUpdateProgress()
  {
    if(updateProgress<98)
	{
	  updateProgress=updateProgress+1;
	  var timer = window.setTimeout('getUpdateProgress()', 750);
    }
    progress.value = updateProgress;
  }
</script>
</body>
</html>)!^!";



int WebOTA::add_http_routes(WebServer *server, const char *path) {
	// Upload firmware page
	server->on(path, HTTP_GET, [server,this]() {
		String html = FPSTR(ota_upload_form);
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
