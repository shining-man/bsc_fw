// Copyright (c) 2022 Tobias Himmler
// 
// This software is released under the MIT License.
// https://opensource.org/licenses/MIT

#include "webUtility.h"
#include "log.h"
#include <SPIFFS.h>
#include <WebServer.h>


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


