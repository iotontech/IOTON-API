// Includes --------------------------------------------------------------------
#include "Ioton.h"


// Constants -------------------------------------------------------------------
#define SERVER_PORT     80	// Server port
#define SERVER_TIMEOUT  5	// Server timeout in seconds in case link breaks

// ############################## IMPORTANT ####################################
// Uncomment the line below once to set up the ESP8266 (mode and acces point), by default it's commented
//#define CONFIG_ESP        // After configuring the ESP8266, leave this commented!
#define SSID    "ssid"      // IMPORTANT: Enter router ssid inside the quotes
#define PASS    "password"  // IMPORTANT: Enter router password inside the quotes


// Private variables -----------------------------------------------------------
DigitalOut  out1(PIN2);
DigitalOut  out2(PIN3);
DigitalIn   in1(PIN13);
DigitalIn   in2(PIN14);
PwmOut      pwm1(PIN9);
PwmOut      pwm2(PIN10);
AnalogIn    ain1(PIN25);
AnalogIn    ain2(PIN26);
Timer t1;
Timer t2;

struct tm t;
volatile unsigned int bufflen;
volatile int replycount, servreq;
volatile int bufl, ipdLen, linkID, webcounter;
char bufBattery[10];
char bufRGB[8] = "000000";
char bufPwm1[5];
char bufPwm2[5];
char bufAin1[5];
char bufAin2[5];
char webcount[8];
char lasthit[30];
char timebuf[30];
char type[16];
char type1[16];
char channel[2];
char cmdbuff[32];
char replybuff[1024];
char webdata[2048]; // This may need to be bigger depending on WEB browser used
char webbuff[4096]; // Increase this if more web page data added
volatile int count, DataRX, weberror;


// Function Prototypes ---------------------------------------------------------
void configESP(void);
void initESP(void);
void sendCMD(void);
void getReply(int timeout, int getcount);
void ReadWebData(void);
void startServer(void);
void sendPage(void);
void SendWEB(void);
void sendCheck(void);
void updateValues(void);
void setRTC(int hour, int minute, int day, int month, int year);


// Serial Interrupt read ESP data ----------------------------------------------
void callback()
{
    while (wifi.readable())
    {
        webbuff[count] = wifi.getc();
        count++;
    }

    if(strlen(webbuff) > bufflen)
    {
        DataRX = 1;
    }
}

// *****************************************************************************
// MAIN PROGRAM ****************************************************************
int main(void)
{
#ifdef CONFIG_ESP
    configESP();    // Run this once to set up the ESP8266 *******************
#endif

    initESP();

    // The main LOOP
    while (1)
    {
        if(DataRX == 1)
        {
            ReadWebData();

            if (servreq == 1 && weberror == 0)
            {
                sendPage();
            }

            wifi.attach(&callback);
            usb.printf(" IPD Data:\r\n\n Link ID = %d,\r\n IPD Header Length = %d \r\n IPD Type = %s\r\n", linkID, ipdLen, type);
            usb.printf("\n\n  HTTP Packet: \n\n%s\n", webdata);
            usb.printf("  Web Characters sent : %d\n\n", bufl);
            usb.printf("  -------------------------------------\n\n");
            strcpy(lasthit, timebuf);
            servreq = 0;
        }
    }   // end of main LOOP
}   // end of main function


// Private functions -----------------------------------------------------------

//  +++ This is for ESP8266 config only, run this once to set up the ESP8266 +++
// See: #define CONFIG_ESP
void configESP(void)
{
    ton.setLED(YELLOW);
    ton.enableWifi();

    usb.printf("---------- Starting ESP Config ----------\r\n\n");
    wait(2);
    usb.printf("---------- Reset & get Firmware ----------\r\n");
    strcpy(cmdbuff,"AT+RST\r\n");
    sendCMD();
    getReply(2000, 500);
    usb.printf(replybuff);
    wait(1);

    usb.printf("\n---------- Get Version ----------\r\n");
    strcpy(cmdbuff,"AT+GMR\r\n");
    sendCMD();
    getReply(1000, 50);
    usb.printf(replybuff);
    wait(1);

    // set CWMODE to 1=Station,2=AP,3=BOTH, default mode 1 (Station)
    usb.printf("\n---------- Setting Mode ----------\r\n");
    strcpy(cmdbuff, "AT+CWMODE=1\r\n");
    sendCMD();
    getReply(1000, 50);
    usb.printf(replybuff);
    wait(1);

    // set CIPMUX to 0=Single,1=Multi
    usb.printf("\n---------- Setting Connection Mode ----------\r\n");
    strcpy(cmdbuff, "AT+CIPMUX=1\r\n");
    sendCMD();
    getReply(1000, 50);
    usb.printf(replybuff);
    wait(1);

    usb.printf("\n---------- Listing Acces Points ----------\r\n");
    strcpy(cmdbuff, "AT+CWLAP\r\n");
    sendCMD();
    getReply(3000, 200);
    usb.printf(replybuff);
    wait(1);

    usb.printf("\n---------- Connecting to AP ----------\r\n");
    usb.printf("ssid = %s   pwd = %s\r\n", SSID, PASS);
    strcpy(cmdbuff, "AT+CWJAP=\"");
    strcat(cmdbuff, SSID);
    strcat(cmdbuff, "\",\"");
    strcat(cmdbuff, PASS);
    strcat(cmdbuff, "\"\r\n");
    usb.printf(cmdbuff);
    sendCMD();
    getReply(8000, 50);
    usb.printf(replybuff);
    wait(5);

    usb.printf("\n---------- Get IP's ----------\r\n");
    strcpy(cmdbuff, "AT+CIFSR\r\n");
    sendCMD();
    getReply(2000, 50);
    usb.printf(replybuff);
    wait(1);

    usb.printf("\n---------- Get Connection Status ----------\r\n");
    strcpy(cmdbuff, "AT+CIPSTATUS\r\n");
    sendCMD();
    getReply(2000, 50);
    usb.printf(replybuff);
    wait(1);

    usb.printf("\n\n\n  If you get a valid IP, ESP8266 has been set up.\r\n");
    usb.printf("  Run this if you want to reconfig the ESP8266 at any time.\r\n");

    wait(2);
}

void initESP(void)
{
    ton.setLED(BLUE);

    wait_ms(500);
    usb.printf("\f\n\r------------ ESP8266 Hardware Start --------------\n\r");
    ton.enableWifi();
    getReply(6000, 500);

    if (time(NULL) < 1420070400)
    {
        // Set default RTC: hour, minute, day, month, year
        setRTC(0, 0, 1, 1, 16);
    }
    wait_ms(60);
    startServer();

    ton.setLED(NONE);
}

// Static WEB page
void sendPage()
{
    updateValues();

    // WEB page data - Header
    strcat(webbuff, "HTTP/1.1 200 OK\r\n");
    strcat(webbuff, "Content-Type:text/html\r\n\r\n");
    strcpy(webbuff, "<!DOCTYPE HTML>\r\n");
    strcat(webbuff, "<html><head><title>ESP8266 TON</title></head>");
    strcat(webbuff, "<body>");
    strcat(webbuff, "<div style=\"text-align:center; background-color:#F4F4F4; color:#00AFEF;\"><h1>IOTON.CC - ESP8266</h1>");
    strcat(webbuff, "Hit Count - ");
    strcat(webbuff, webcount);
    strcat(webbuff, "<br>Last Hit - ");
    strcat(webbuff, lasthit);
    strcat(webbuff, "</div><br /><hr>");

    // RTC Time
    strcat(webbuff, "<h3>TON RTC Time -&nbsp&nbsp");
    strcat(webbuff, timebuf);
    strcat(webbuff, "</h3>\r\n");

    // Battery voltage
    strcat(webbuff, "<p><form action=\"\" method=\"POST\" ><strong> Battery:&nbsp&nbsp<input type=\"text\" size=6 value=\"");
    strcat(webbuff, bufBattery);
    strcat(webbuff, "\"> V");

    // LED RGB
    strcat(webbuff, "<p><strong> LED RGB color code:&nbsp&nbsp #<input type=\"text\" name=\"rgb\" size=6 value=\"");
    strcat(webbuff, bufRGB);
    strcat(webbuff, "\">");

    // Output1
    if(out1 == 0)
    {
        strcat(webbuff, "<p><input type=\"radio\" name=\"out1\" value=\"0\" checked>  Digital Out 1  off");
        strcat(webbuff, "<br><input type=\"radio\" name=\"out1\" value=\"1\" >  Digital Out 1 on");
    }
    else
    {
        strcat(webbuff, "<p><input type=\"radio\" name=\"out1\" value=\"0\" >  Digital Out 1 off");
        strcat(webbuff, "<br><input type=\"radio\" name=\"out1\" value=\"1\" checked>  Digital Out 1 on");
    }

    // Output2
    if(out2 == 0)
    {
        strcat(webbuff, "<p><input type=\"radio\" name=\"out2\" value=\"0\" checked>  Digital Out 2 off");
        strcat(webbuff, "<br><input type=\"radio\" name=\"out2\" value=\"1\" >  Digital Out 2 on");
    }
    else
    {
        strcat(webbuff, "<p><input type=\"radio\" name=\"out2\" value=\"0\" >  Digital Out 2 off");
        strcat(webbuff, "<br><input type=\"radio\" name=\"out2\" value=\"1\" checked>  Digital Out 2 on");
    }

    // Input1
    if(in1 == 0)
    {
        strcat(webbuff, "<p><input type=\"radio\" name=\"in1\" value=\"0\" >  Digital In 1");
    }
    else
    {
        strcat(webbuff, "<p><input type=\"radio\" name=\"in1\" value=\"1\" checked>  Digital In 1");
    }

    // Input2
    if(in2 == 0)
    {
        strcat(webbuff, "<p><input type=\"radio\" name=\"in2\" value=\"0\" >  Digital In 2");
    }
    else
    {
        strcat(webbuff, "<p><input type=\"radio\" name=\"in2\" value=\"1\" checked>  Digital In 2");
    }

    // PWM1
    strcat(webbuff, "<p><strong> PWM1 (0 - 100):&nbsp&nbsp<input type=\"text\" name=\"pwm1\" size=5 value=\"");
    strcat(webbuff, bufPwm1);
    strcat(webbuff, "\">");

    // PWM2
    strcat(webbuff, "<p><strong> PWM2 (0 - 100):&nbsp&nbsp<input type=\"text\" name=\"pwm2\" size=5 value=\"");
    strcat(webbuff, bufPwm2);
    strcat(webbuff, "\">");

    // Ain1
    strcat(webbuff, "<p><form method=\"POST\"> <strong> Ain1:&nbsp&nbsp<input type=\"text\" size=5 value=\"");
    strcat(webbuff, bufAin1);
    strcat(webbuff, "\">");

    // Ain2
    strcat(webbuff, "<p><form method=\"POST\"> <strong> Ain2:&nbsp&nbsp<input type=\"text\" size=5 value=\"");
    strcat(webbuff, bufAin2);
    strcat(webbuff, "\">");

    strcat(webbuff, "</strong><p><input type=\"submit\" value=\"send-refresh\" </form>");
    strcat(webbuff, "<p/><h2>How to use:</h2><ul>");
    strcat(webbuff, "<li>Set color code and PWM duty-cycles.</li>");
    strcat(webbuff, "<li>Select the Radio buttons to control the digital out pins.</li>");
    strcat(webbuff, "<li>Click 'Send-Refresh' to send.</li>");
    strcat(webbuff, "<li>Use the 'Send-Refresh' button to refresh the data.</li>");
    strcat(webbuff, "</ul>");
    strcat(webbuff, "</body></html>");
    // end of WEB page data

    bufl = strlen(webbuff); // get total page buffer length
    sprintf(cmdbuff,"AT+CIPSEND=%d,%d\r\n", linkID, bufl); // send IPD link channel and buffer character length.
    usb.printf(cmdbuff);
    sendCMD();
    getReply(200, 7);
    wait_ms(60);
    SendWEB();  // send web page
    memset(webbuff, '\0', sizeof(webbuff));
    sendCheck();
}

//  Wait for ESP "SEND OK" reply, then close IP to load web page
void sendCheck()
{
    weberror=1;
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

    if(weberror == 1)
    { // restart connection
        strcpy(cmdbuff, "AT+CIPMUX=1\r\n");
        sendCMD();
        getReply(500, 10);
        usb.printf(replybuff);
        sprintf(cmdbuff,"AT+CIPSERVER=1,%d\r\n", SERVER_PORT);
        sendCMD();
        getReply(500, 10);
        usb.printf(replybuff);
    }
    else
    { // close current connection
        sprintf(cmdbuff, "AT+CIPCLOSE=%s\r\n", channel);
        sendCMD();
        getReply(500, 24);
        usb.printf(replybuff);
    }

    t2.stop();
}

// Reads and processes GET and POST web data
void ReadWebData()
{
    wait_ms(500);
    wifi.attach(NULL);
    count = 0;
    DataRX = 0;
    weberror = 0;
    memset(webdata, '\0', sizeof(webdata));
    int x = strcspn (webbuff,"+");

    if(x)
    {
        strcpy(webdata, webbuff + x);
        weberror = 0;
        sscanf(webdata,"+IPD,%d,%d:%s", &linkID, &ipdLen, type);

        sprintf(channel, "%d",linkID);
        if (strstr(webdata, "GET") != NULL)
        {
            servreq = 1;
        }
        if (strstr(webdata, "POST") != NULL)
        {
            servreq = 1;

            char* full_content;
            full_content = strstr(webbuff, "rgb=") + 4;

            int v_out1 = 0, v_out2 = 0, v_in1 = 0, v_in2 = 0, v_pwm1 = 0, v_pwm2 = 0;
            char* part_content;
            part_content = strtok(full_content,"&");
            strcpy(bufRGB, part_content);
            ton.setLED(bufRGB);

            part_content = strtok(NULL,"&");
            sscanf(part_content,"out1=%d", &v_out1);

            part_content = strtok(NULL,"&");
            sscanf(part_content,"out2=%d", &v_out2);

            part_content = strtok(NULL,"&");
            sscanf(part_content,"in1=%d", &v_in1);

            part_content = strtok(NULL,"&");
            sscanf(part_content,"in2=%d", &v_in2);

            part_content = strtok(NULL,"&");
            sscanf(part_content,"pwm1=%d", &v_pwm1);

            part_content = strtok(NULL,"&");
            sscanf(part_content,"pwm2=%d", &v_pwm2);

            out1 = v_out1; out2 = v_out2;
            pwm1 = v_pwm1 / 100.0f; pwm2 = v_pwm2 / 100.0f;
        }
        webcounter++;
        sprintf(webcount, "%d",webcounter);
    }
    else
    {
        memset(webbuff, '\0', sizeof(webbuff));
        wifi.attach(&callback);
        weberror = 1;
    }
}

// Starts and restarts webserver if errors detected.
void startServer()
{
    updateValues();

    usb.printf("\n\n RTC time   %s\r\n\n",timebuf);
    usb.printf("++++++++++ Resetting ESP ++++++++++\r\n");
    strcpy(cmdbuff,"AT+RST\r\n");
    sendCMD();
    getReply(8000, 1000);
    usb.printf(replybuff);
    usb.printf("%d",count);

    if (strstr(replybuff, "OK") != NULL)
    {
        usb.printf("\n++++++++++ Starting Server ++++++++++\r\n");
        strcpy(cmdbuff, "AT+CIPMUX=1\r\n");  // set multiple connections.
        sendCMD();
        getReply(500, 20);
        usb.printf(replybuff);
        wait(1);

        sprintf(cmdbuff,"AT+CIPSERVER=1,%d\r\n", SERVER_PORT);
        sendCMD();
        getReply(500, 20);
        usb.printf(replybuff);
        wait(1);

        sprintf(cmdbuff,"AT+CIPSTO=%d\r\n", SERVER_TIMEOUT);
        sendCMD();
        getReply(500, 20);
        usb.printf(replybuff);
        wait(5);

        usb.printf("\n Getting Server IP \r\n");
        strcpy(cmdbuff, "AT+CIFSR\r\n");

        while(weberror == 0)
        {
            sendCMD();
            getReply(1000, 50);
            usb.printf(replybuff);
            if (strstr(replybuff, "0.0.0.0") == NULL)
            {
                weberror = 1;   // wait for valid IP
            }
        }

        usb.printf("\n Enter WEB address (IP) found below in your browser \r\n\n");
        usb.printf("\n The MAC address is also shown below,if it is needed \r\n\n");
        replybuff[strlen(replybuff)-1] = '\0';
        sprintf(webdata,"%s", replybuff);
        usb.printf(webdata);
        bufflen = 200;
        count = 0;
        usb.printf("\n\n++++++++++ Ready ++++++++++\r\n\n");
        wifi.attach(&callback);
    }
    else
    {
        usb.printf("\n++++++++++ ESP8266 error, check power/connections ++++++++++\r\n");
        while(1)
        {
            ton.setLED(RED);
            wait(1);
            ton.setLED(NONE);
            wait(1);
        }
    }

    wait_ms(60);
}

// ESP Command data send
void sendCMD()
{
    wifi.printf("%s", cmdbuff);
}

// Large WEB buffer data send
void SendWEB()
{
    volatile int i = 0;

    if(wifi.writeable())
    {
        while(webbuff[i] != '\0')
        {
            wifi.putc(webbuff[i]);
            i++;
        }
    }
}

// Get Command and ESP status replies
void getReply(int timeout, int getcount)
{
    memset(replybuff, '\0', sizeof(replybuff));
    t1.reset();
    t1.start();
    replycount = 0;

    while(t1.read_ms() < timeout && replycount < getcount)
    {
        if(wifi.readable())
        {
            replybuff[replycount] = wifi.getc();
            replycount++;
        }
    }

    t1.stop();
}

// Update variables and get RTC time
void updateValues(void)
{
    sprintf(bufBattery, "%2.3f", ton.getBattery());

    sprintf(bufPwm1, "%d", (int)(pwm1 * 100.0f));
    sprintf(bufPwm2, "%d", (int)(pwm2 * 100.0f));

    sprintf(bufAin1, "%d", (int)(ain1 * 100.0f));
    sprintf(bufAin2, "%d", (int)(ain2 * 100.0f));

    time_t seconds = time(NULL);
    strftime(timebuf,50,"%H:%M:%S %a %d %b %y", localtime(&seconds));
}

// Set RTC time
void setRTC(int hour, int minute, int day, int month, int year)
{
    t.tm_sec = (0);             // 0-59
    t.tm_min = (minute);        // 0-59
    t.tm_hour = (hour);         // 0-23
    t.tm_mday = (day);          // 1-31
    t.tm_mon = (month-1);       // 0-11  "0" = Jan, -1 added for Mbed RCT clock format
    t.tm_year = ((year)+100);   // year since 1900,  current DCF year + 100 + 1900 = correct year
    set_time(mktime(&t));       // set RTC clock
}
