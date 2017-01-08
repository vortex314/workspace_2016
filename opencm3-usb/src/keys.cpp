#include <EventBus.h>

const char* hash2string(uint16_t hash)
{
    switch(hash)
    {
	case EB_DST :  return "dst";
    case EB_SRC :  return "src";
    case EB_EVENT :  return "event";
    case EB_REQUEST :  return "request";
    case EB_REPLY :  return "reply";
case H("Actor") : return "Actor";
case H("blinkFast") : return "blinkFast";
case H("blinkSlow") : return "blinkSlow";
case H("connect") : return "connect";
case H("connected") : return "connected";
case H("data") : return "data";
case H("disconnect") : return "disconnect";
case H("disconnected") : return "disconnected";
case H("Echo") : return "Echo";
case H("error") : return "error";
case H("from") : return "from";
case H("host") : return "host";
case H("id") : return "id";
case H("Led") : return "Led";
case H("line") : return "line";
case H("log") : return "log";
case H("Logger") : return "Logger";
case H("memory") : return "memory";
case H("message") : return "message";
case H("mqtt") : return "mqtt";
case H("MqttCl") : return "MqttCl";
case H("off") : return "off";
case H("on") : return "on";
case H("ping") : return "ping";
case H("port") : return "port";
case H("publish") : return "publish";
case H("published") : return "published";
case H("Relay") : return "Relay";
case H("relay1") : return "relay1";
case H("relay2") : return "relay2";
case H("relay3") : return "relay3";
case H("relay4") : return "relay4";
case H("reset") : return "reset";
case H("Router") : return "Router";
case H("rxd") : return "rxd";
case H("serial") : return "serial";
case H("setup") : return "setup";
case H("slip") : return "slip";
case H("state") : return "state";
case H("stm32") : return "stm32";
case H("subscribe") : return "subscribe";
case H("switch") : return "switch";
case H("sys") : return "sys";
case H("system") : return "system";
case H("Tester") : return "Tester";
case H("time") : return "time";
case H("timeout") : return "timeout";
case H("to") : return "to";
case H("topic") : return "topic";
case H("uint32_t") : return "uint32_t";
case H("Usb") : return "Usb";
case H("usb.rxd") : return "usb.rxd";
default : {
		return 0;
        }
    }
}
