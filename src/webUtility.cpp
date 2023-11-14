// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "webUtility.h"
#include "log.h"
#include <SPIFFS.h>
#include <WebServer.h>
#include "defines.h"

static const char * TAG = "WEB";

String getContentType(WebServer *server, String filename)
{
  if (server->hasArg("download")) {
    return "application/octet-stream";
  } else if (filename.endsWith(".htm")) {
    return "text/html";
  } else if (filename.endsWith(".html")) {
    return "text/html";
  } else if (filename.endsWith(".css")) {
    return "text/css";
  } else if (filename.endsWith(".js")) {
    return "application/javascript";
  } else if (filename.endsWith(".png")) {
    return "image/png";
  } else if (filename.endsWith(".gif")) {
    return "image/gif";
  } else if (filename.endsWith(".jpg")) {
    return "image/jpeg";
  } else if (filename.endsWith(".ico")) {
    return "image/x-icon";
  } else if (filename.endsWith(".xml")) {
    return "text/xml";
  } else if (filename.endsWith(".pdf")) {
    return "application/x-pdf";
  } else if (filename.endsWith(".zip")) {
    return "application/x-zip";
  } else if (filename.endsWith(".gz")) {
    return "application/x-gzip";
  }
  return "text/plain";
}

bool exists(String path)
{
  bool yes = false;
  File file = SPIFFS.open(path, "r");
  if(!file.isDirectory())
  {
    yes = true;
  }
  file.close();
  return yes;
}

bool handleFileRead(WebServer *server, String path)
{
  if (path.endsWith("/"))
  {
    path += "index.htm";
  }
  String contentType = getContentType(server, path);
  String pathWithGz = path + ".gz";
  if (exists(pathWithGz) || exists(path))
  {
    if (exists(pathWithGz))
    {
      path += ".gz";
    }
    File file = SPIFFS.open(path, "r");
    server->streamFile(file, contentType);
    file.close();
    return true;
  }
  return false;
}


void handleFileUpload(WebServer *server, String fileName) 
{
  static File fsUploadFile;
  HTTPUpload& upload = server->upload();
  if (upload.status == UPLOAD_FILE_START)
  {
    //if (upload.filename.length() > 30)
    //{
    //  upload.filename = upload.filename.substring(upload.filename.length() - 30, upload.filename.length());  // Dateinamen auf 30 Zeichen kürzen
    //}
    upload.filename=fileName;
    BSC_LOGI(TAG,"handleFileUpload Name: /%s", upload.filename.c_str());
    fsUploadFile = SPIFFS.open("/" + server->urlDecode(upload.filename), "w");
  } else if (upload.status == UPLOAD_FILE_WRITE)
  {
    //BSC_LOGI(TAG,"handleFileUpload Data: %u\n", upload.currentSize);
    if (fsUploadFile)
      fsUploadFile.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END)
  {
    if (fsUploadFile) fsUploadFile.close();
    //BSC_LOGI(TAG,"handleFileUpload Size: %u\n", upload.totalSize);
    BSC_LOGI(TAG,"handleFileUpload finish");
  }
}

