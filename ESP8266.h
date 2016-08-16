#ifndef ESP8266_H
#define ESP8266_H
#include "mbed.h"

/**
* An interface to the ESP8266
*/
class ESP8266 {
public:

    /**
    * Create an interface to the ESP8266
    *
    * @param tx UART TX line pin
    * @param rx UART RX line pin
    */
    ESP8266(PinName tx, PinName rx) : _serial(tx, rx)
    {
        _serial.baud(115200);
        _connected = false;
    }


    bool connect(const char* ssid, const char* pass, uint8_t mode = 0)
    {
        sprintf(cmdbuff,"AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, pass);
        sendCMD();
        getReply(8000, 50);

        if (strstr(replybuff, "OK") == NULL) return false;

        sprintf(cmdbuff,"AT+CIPMUX=%d\r\n", mode);
        sendCMD();
        getReply(500, 20);

        _connected = true;
        return true;
    }

    char* httpGet(const char* host, const char* url, int port = 80)
    {
        connectToServer(host, port);

        sprintf(webbuff,
            "GET %s HTTP/1.1\r\nHost: %s\r\nAccept: */*\r\nConnection: close\r\n\r\n",
            url, host);

        sendWebBuff();

        return replybuff;
    }

    // Send data to the ThingSpeak
    bool sendThingSpeak(char* data, const char* apikey)
    {
        connectToServer("184.106.153.149", 80);

        sprintf(webbuff, "GET /update?key=%s&%s\r\n", apikey, data);
        sprintf(cmdbuff,"AT+CIPSEND=%d\r\n", strlen(webbuff)); // send buffer character length.
        sendCMD();
        getReply(200, 50);
        SendWEB();  // send data
        memset(webbuff, '\0', sizeof(webbuff));

        volatile int weberror = 1;
        t2.reset();
        t2.start();

        while(weberror == 1 && t2.read() < 5)
        {
            getReply(500, 24);
            if (strstr(replybuff, "OK") != NULL)
            {
                weberror=0;   // wait for valid SEND OK
            }
        }

        t2.stop();

        if(weberror == 1)
        { // error
            return false;
        }
        else
        { // success
            return true;
        }
    }

    bool isConnected(void)
    {
        return _connected;
    }

private:

    Serial _serial;
    volatile bool _connected;

    Timer t1;
    Timer t2;
    char cmdbuff[64];
    char webbuff[1024];
    char replybuff[1024];

    // Connect to host via TCP
    void connectToServer(const char* host, int port)
    {
        sprintf(cmdbuff,"AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", host, port);
        sendCMD();
        getReply(2000, 50);
    }

    bool sendWebBuff(void)
    {
        sprintf(cmdbuff,"AT+CIPSEND=%d\r\n", strlen(webbuff)); // send buffer character length.
        sendCMD();
        getReply(200, 50);
        SendWEB();  // send data
        memset(webbuff, '\0', sizeof(webbuff));
        return sendCheck();
    }

    //  Wait for ESP "SEND OK" reply
    bool sendCheck()
    {
        volatile int weberror = 1;
        t2.reset();
        t2.start();

        while(weberror == 1 && t2.read() < 5)
        {
            getReply(500, 24);
            if (strstr(replybuff, "SEND OK") != NULL)
            {
                weberror=0;   // wait for valid SEND OK
            }
        }
        t2.stop();

        if(weberror == 1)
        { // error
            strcpy(cmdbuff, "AT+CIPMUX=0\r\n");
            sendCMD();
            getReply(500, 10);

            return false;
        }
        else
        { // close current connection
            sprintf(cmdbuff, "AT+CIPCLOSE\r\n");
            sendCMD();
            getReply(500, 600);

            return true;
        }
    }

    // ESP Command data send
    void sendCMD()
    {
        _serial.printf("%s", cmdbuff);
    }

    // Large WEB buffer data send
    int SendWEB()
    {
        volatile int i = 0;

        if(_serial.writeable())
        {
            while(webbuff[i] != '\0')
            {
                _serial.putc(webbuff[i]);
                i++;
            }
        }

        return i;
    }

    // Get Command and ESP status replies
    void getReply(int timeout, int getcount)
    {
        volatile int replycount = 0;

        memset(replybuff, '\0', sizeof(replybuff));
        t1.reset();
        t1.start();

        while(t1.read_ms() < timeout && replycount < getcount)
        {
            if(_serial.readable())
            {
                replybuff[replycount] = _serial.getc();
                replycount++;
            }
        }

        t1.stop();
    }
};

#endif
