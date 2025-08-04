#include "OTAupdater.h"
#include <Arduino.h>
#include <Update.h>
#include "log.h"
#include "defines.h"

static const char *TAG = "OTA";

OTAupdater otaUpdater;

#define CHUNK_SIZE 51200

const char uploadFormV1[] PROGMEM = R"!^!(
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
    <span class='btnBack' onclick='history.back()'><img src="/back.svg" alt="&#10094;" width="21" height="21"></span>
    <span class='hl'>Web-Update</span>
  </div>
  <div class="content">
	<div><u>Installierte FW-Version:</u> <span id='FwVersion'></span><br>
  <span id='FwVersionHinweis'></span></div><br>
	<p><b>Aktuelles verfügbares Release (github)</b>
	<div><u>FW-Version:</u> <span id='gitFwVersion'></span></div><br>
	<div><u>Veröffentlicht am:</u> <span id='gitFwPublishedAt'></span></div><br>
	<div id='fitFwDescLabel'><u>Beschreibung:</u><br><span id='fitFwDesc'></span></div><br>
	</p>
	<hr><br>

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
	__readLastFwVersionGithub();

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

  function __readLastFwVersionGithub()
  {
    var __fwVersion="";
    var __fwVersionNow="";
    var __description="";
    var __published_at="";

    fetch('https://api.github.com/repos/shining-man/bsc_fw/releases/latest')
    .then(__response => __response.json())
    .then((__data) => {
  	  __fwVersion=__data.name.split("_")[0].toLowerCase();
      __published_at=__data.published_at.replace("T"," ").replace("Z","");
      __description=__data.body.replaceAll("\r\n", "<br>");
      __description=__description.replaceAll("\n", "<br>");

      document.getElementById('gitFwVersion').innerHTML =__fwVersion;
      document.getElementById('gitFwPublishedAt').innerHTML =__published_at;

      fetch('/restapi')
      .then(__response => __response.json())
      .then((__data) => {
        __fwVersionNow = __data.system.fw_version.toLowerCase();
        document.getElementById('FwVersion').innerHTML = __fwVersionNow;
        
        console.log(__fwVersionNow);
        console.log(__fwVersion);

        if(__fwVersion != __fwVersionNow){ 
          document.getElementById('fitFwDesc').innerHTML = __description;
        } else {
          document.getElementById('FwVersionHinweis').innerHTML = "Es ist keine neue Version verfügbar!";
          document.getElementById('fitFwDescLabel').innerHTML = "";
        } 
      });
    });
  }
</script>
</body>
</html>)!^!";

// 	<div><a onclick="window.open('https://github.com/shining-man/bsc_fw/releases/latest/download/fw_bsc_ota.bin','_blank');" href="#">Download from GitHub</a></div>

bool OTAupdater::init(WebServer *server, WebSettings *webSettings, const char *path, bool enUpdatePage)
{
	if(this->isInit) return false;
	setHttpRoutes(server, webSettings, path, enUpdatePage);
	this->isInit = true;

	return true;
}

void OTAupdater::delayWithHandleClient(WebServer *server, uint16_t delay_ms) {
  uint32_t u32_startMillis = millis();

  while((millis()-u32_startMillis)<delay_ms)
  {
	server->handleClient();
	vTaskDelay(5/portTICK_PERIOD_MS);
  }
}

void OTAupdater::setHttpRoutes(WebServer *server, WebSettings *webSettings, const char *path, bool enUpdatePage)
{
	// Upload firmware page
	if(enUpdatePage)
	{
		server->on(path, HTTP_GET, [server, webSettings, this]()
		{
      if(!performAuthentication(server, webSettings)) return;
			String html = FPSTR(uploadFormV1);
			server->send_P(200, "text/html", html.c_str());
		});
	}

	// Handling uploading firmware file
	server->on(path, HTTP_POST, [server, webSettings, this]()
	{
    if(!performAuthentication(server, webSettings)) return;

		server->send(200, "text/plain", (Update.hasError()) ? "Update: fail\n" : "Update: OK\n");
		delayWithHandleClient(server, 1000);
		ESP.restart();
	},
	[server, webSettings, this]()
	{
    if(!performAuthentication(server, webSettings)) return;

		HTTPUpload& upload = server->upload();

		if(upload.status == UPLOAD_FILE_START)
		{
			BSC_LOGI(TAG,"Firmware update initiated: %s",upload.filename.c_str());
			uint32_t sketchSize = (ESP.getFreeSketchSpace()-0x1000) & 0xFFFFF000;

			if(!Update.begin(sketchSize))
			{
				BSC_LOGI(TAG,"Firmware update:", Update.errorString());
			}
		}
		else if(upload.status == UPLOAD_FILE_WRITE)
		{
			if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) // if error
			{
				BSC_LOGI(TAG,"Firmware update:", Update.errorString());
			}

			// Print info all 100k
			static uint32_t nextInfoSize = CHUNK_SIZE;
			if(upload.totalSize >= nextInfoSize)
			{
				BSC_LOGI(TAG,"%d k ",nextInfoSize/1024);
				nextInfoSize += CHUNK_SIZE;
			}
		}
		else if(upload.status == UPLOAD_FILE_END)
		{
			if(Update.end(true))
			{
				BSC_LOGI(TAG,"Firmware update successful: %u bytes;\nRebooting...", upload.totalSize);
			}
			else
			{
				BSC_LOGI(TAG,"Firmware update:", Update.errorString());
			}
		}
	});
}
