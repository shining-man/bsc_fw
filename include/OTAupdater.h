#include <Arduino.h>
#include <WebServer.h>

class OTAupdater {
	public:
		bool init(WebServer *server, const char *path, bool enUpdatePage);

	private:
		bool isInit;

		void delayWithHandleClient(WebServer *server, uint16_t delay_ms);
		void setHttpRoutes(WebServer *server, const char *path, bool enUpdatePage);
};

extern OTAupdater otaUpdater;
