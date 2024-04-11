#include <Arduino.h>
#include <WebServer.h>
#include <WebSettings.h>
#include <WebUtility.h>

class OTAupdater {
	public:
		bool init(WebServer *server, WebSettings *webSettings, const char *path, bool enUpdatePage);

	private:
		bool isInit;

		void delayWithHandleClient(WebServer *server, uint16_t delay_ms);
		void setHttpRoutes(WebServer *server, WebSettings *webSettings, const char *path, bool enUpdatePage);
};

extern OTAupdater otaUpdater;
