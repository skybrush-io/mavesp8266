/****************************************************************************
 *
 * Copyright (c) 2015, 2016 Gus Grubba. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavesp8266_httpd.cpp
 * ESP8266 Wifi AP, MavLink UART/UDP Bridge
 *
 * @author Gus Grubba <mavlink@grubba.com>
 */

#include "mavesp8266.h"
#include "mavesp8266_component.h"
#include "mavesp8266_httpd.h"
#include "mavesp8266_parameters.h"
#include "mavesp8266_gcs.h"
#include "mavesp8266_vehicle.h"
#include "mavesp8266_power_mgmt.h"

#if MAVESP8266_IS_ESP32
#  include <Update.h>
#  include <WebServer.h>
#  include <esp_spi_flash.h>
#else
#  include <ESP8266WebServer.h>
#  define WebServer ESP8266WebServer
#endif

const char PROGMEM kTEXTPLAIN[]  = "text/plain";
const char PROGMEM kTEXTHTML[]   = "text/html";
const char PROGMEM kTEXTCSS[]    = "text/css";
const char PROGMEM kACCESSCTL[]  = "Access-Control-Allow-Origin";
const char PROGMEM kUPLOADFORM[] =
    "<form method=\"POST\" action=\"/upload\" enctype=\"multipart/form-data\">\n"
    "<p><label>Firmware to upload</label>\n"
    "<input type=\"file\" name=\"update\" /></p>\n"
    "<input type=\"submit\" value=\"Update\" />\n"
    "</form>\n"
    "<p>Update will take about 30 seconds, and you will need to reconnect.</p>\n"
    "<p>If parameters are not compatible then the default SSID of <code>ArduPilot</code>\n"
    "with password <code>ardupilot</code> will be used.</p>\n";
const char PROGMEM kHEADER[]     =
    "<!doctype html><html><head>\n"
    "<meta charset=\"UTF-8\">\n"
    "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n"
    "<title>MavLink Bridge</title>\n"
    "<link rel=\"stylesheet\" href=\"/style.css\">\n"
    "<link rel=\"stylesheet\" href=\"/custom.css\">\n"
    "</head><body>\n"
    "<header>\n"
    "<h1>MAVLink WiFi Bridge</h1>\n"
    "<nav id=\"main-nav\">\n"
    "<a href=\"/\">Home</a>\n"
    "<a href=\"/getstatus\">Status</a>\n"
    "<a href=\"/setup\">Setup</a>\n"
    "<a href=\"/update\">Update</a>\n"
    "<a href=\"/reboot\" onclick=\"return confirm('Are you sure?')\">Reboot</a>\n"
    "</nav>\n"
    "</header>\n"
    "<main>\n";
const char PROGMEM kFOOTER[]     =
    "</main>\n"
    "<script>\n"
    "for (const child of document.getElementById('main-nav').children) {\n"
    "  if (child.tagName === 'A' && child.href === window.location.href) {\n"
    "    child.classList.add('current');\n"
    "  }\n"
    "}\n"
    "</script>\n"
    "<footer>\n"
    "<p>This page uses <a href=\"https://simplecss.org/\">Simple.css</a> by "
    "<a href=\"https://kevq.uk\">Kev Quirk</a></p>\n"
    "</footer></body></html>";
const char PROGMEM kBADARG[]     = "BAD ARGS";
const char PROGMEM kAPPJSON[]    = "application/json";
const char PROGMEM kSTYLESHEET[] = ":root{--sans-font:-apple-system,BlinkMacSystemFont,\"Avenir Next\",Avenir,\"Nimbus Sans L\",Roboto,\"Noto Sans\",\"Segoe UI\",Arial,Helvetica,\"Helvetica Neue\",sans-serif;--mono-font:Consolas,Menlo,Monaco,\"Andale Mono\",\"Ubuntu Mono\",monospace;--bg:#fff;--accent-bg:#f5f7ff;--text:#212121;--text-light:#585858;--border:#898EA4;--accent:#0d47a1;--code:#d81b60;--preformatted:#444;--marked:#ffdd33;--disabled:#efefef}@media (prefers-color-scheme:dark){:root{color-scheme:dark;--bg:#212121;--accent-bg:#2b2b2b;--text:#dcdcdc;--text-light:#ababab;--accent:#ffb300;--code:#f06292;--preformatted:#ccc;--disabled:#111}img,video{opacity:.8}}*,::after,::before{box-sizing:border-box}input,progress,select,textarea{appearance:none;-webkit-appearance:none;-moz-appearance:none}html{font-family:var(--sans-font);scroll-behavior:smooth}body{color:var(--text);background-color:var(--bg);font-size:1.15rem;line-height:1.5;display:grid;grid-template-columns:1fr min(45rem,90%) 1fr;margin:0}body>*{grid-column:2}body>header{background-color:var(--accent-bg);border-bottom:1px solid var(--border);text-align:center;padding:0 .5rem 2rem .5rem;grid-column:1/-1}body>header h1{max-width:1200px;margin:1rem auto}body>header p{max-width:40rem;margin:1rem auto}main{padding-top:1.5rem}body>footer{margin-top:4rem;padding:2rem 1rem 1.5rem 1rem;color:var(--text-light);font-size:.9rem;text-align:center;border-top:1px solid var(--border)}h1{font-size:3rem}h2{font-size:2.6rem;margin-top:3rem}h3{font-size:2rem;margin-top:3rem}h4{font-size:1.44rem}h5{font-size:1.15rem}h6{font-size:.96rem}h1,h2,h3,h4,h5,h6,p{overflow-wrap:break-word}h1,h2,h3{line-height:1.1}@media only screen and (max-width:720px){h1{font-size:2.5rem}h2{font-size:2.1rem}h3{font-size:1.75rem}h4{font-size:1.25rem}}a,a:visited{color:var(--accent)}a:hover{text-decoration:none}[role=button],button,input[type=button],input[type=reset],input[type=submit],label[type=button]{border:none;border-radius:5px;background-color:var(--accent);font-size:1rem;color:var(--bg);padding:.7rem .9rem;margin:.5rem 0}[role=button][aria-disabled=true],button[disabled],input[type=button][disabled],input[type=checkbox][disabled],input[type=radio][disabled],input[type=reset][disabled],input[type=submit][disabled],select[disabled]{cursor:not-allowed}button[disabled],input:disabled,select:disabled,textarea:disabled{cursor:not-allowed;background-color:var(--disabled);color:var(--text-light)}input[type=range]{padding:0}abbr[title]{cursor:help;text-decoration-line:underline;text-decoration-style:dotted}[role=button]:not([aria-disabled=true]):hover,button:enabled:hover,input[type=button]:enabled:hover,input[type=reset]:enabled:hover,input[type=submit]:enabled:hover,label[type=button]:hover{filter:brightness(1.4);cursor:pointer}button:focus-visible:where(:enabled,[role=button]:not([aria-disabled=true])),input:enabled:focus-visible:where([type=submit],[type=reset],[type=button]){outline:2px solid var(--accent);outline-offset:1px}header>nav{font-size:1rem;line-height:2;padding:1rem 0 0 0}header>nav ol,header>nav ul{align-content:space-around;align-items:center;display:flex;flex-direction:row;flex-wrap:wrap;justify-content:center;list-style-type:none;margin:0;padding:0}header>nav ol li,header>nav ul li{display:inline-block}header>nav a,header>nav a:visited{margin:0 .5rem 1rem .5rem;border:1px solid var(--border);border-radius:5px;color:var(--text);display:inline-block;padding:.1rem 1rem;text-decoration:none}header>nav a:hover{border-color:var(--accent);color:var(--accent);cursor:pointer}@media only screen and (max-width:720px){header>nav a{border:none;padding:0;text-decoration:underline;line-height:1}}aside,details,pre,progress{background-color:var(--accent-bg);border:1px solid var(--border);border-radius:5px;margin-bottom:1rem}aside{font-size:1rem;width:30%;padding:0 15px;margin-left:15px;float:right}@media only screen and (max-width:720px){aside{width:100%;float:none;margin-left:0}}article,fieldset{border:1px solid var(--border);padding:1rem;border-radius:5px;margin-bottom:1rem}article h2:first-child,section h2:first-child{margin-top:1rem}section{border-top:1px solid var(--border);border-bottom:1px solid var(--border);padding:2rem 1rem;margin:3rem 0}section+section,section:first-child{border-top:0;padding-top:0}section:last-child{border-bottom:0;padding-bottom:0}details{padding:.7rem 1rem}summary{cursor:pointer;font-weight:700;padding:.7rem 1rem;margin:-.7rem -1rem;word-break:break-all}details[open]>summary+*{margin-top:0}details[open]>summary{margin-bottom:.5rem}details[open]>:last-child{margin-bottom:0}table{border-collapse:collapse;display:block;margin:1.5rem 0;overflow:auto;width:100%}td,th{border:1px solid var(--border);text-align:left;padding:.5rem}th{background-color:var(--accent-bg);font-weight:700}tr:nth-child(even){background-color:var(--accent-bg)}table caption{font-weight:700;margin-bottom:.5rem}input,select,textarea{font-size:inherit;font-family:inherit;padding:.5rem;margin-bottom:.5rem;color:var(--text);background-color:var(--bg);border:1px solid var(--border);border-radius:5px;box-shadow:none;max-width:100%;display:inline-block}label{display:block}textarea:not([cols]){width:100%}select:not([multiple]){background-image:linear-gradient(45deg,transparent 49%,var(--text) 51%),linear-gradient(135deg,var(--text) 51%,transparent 49%);background-position:calc(100% - 15px),calc(100% - 10px);background-size:5px 5px,5px 5px;background-repeat:no-repeat;padding-right:25px}input[type=checkbox],input[type=radio]{vertical-align:middle;position:relative;width:min-content}input[type=checkbox]+label,input[type=radio]+label{display:inline-block}input[type=radio]{border-radius:100%}input[type=checkbox]:checked,input[type=radio]:checked{background-color:var(--accent)}input[type=checkbox]:checked::after{content:\" \";width:.18em;height:.32em;border-radius:0;position:absolute;top:.05em;left:.17em;background-color:transparent;border-right:solid var(--bg) .08em;border-bottom:solid var(--bg) .08em;font-size:1.8em;transform:rotate(45deg)}input[type=radio]:checked::after{content:\" \";width:.25em;height:.25em;border-radius:100%;position:absolute;top:.125em;background-color:var(--bg);left:.125em;font-size:32px}@media only screen and (max-width:720px){input,select,textarea{width:100%}}input[type=color]{height:2.5rem;padding:.2rem}input[type=file]{border:0}hr{border:none;height:1px;background:var(--border);margin:1rem auto}mark{padding:2px 5px;border-radius:4px;background-color:var(--marked)}img,video{max-width:100%;height:auto;border-radius:5px}figure{margin:0;text-align:center}figcaption{font-size:.9rem;color:var(--text-light);margin-bottom:1rem}blockquote{margin:2rem 0 2rem 2rem;padding:.4rem .8rem;border-left:.35rem solid var(--accent);color:var(--text-light);font-style:italic}cite{font-size:.9rem;color:var(--text-light);font-style:normal}dt{color:var(--text-light)}code,kbd,pre,pre span,samp{font-family:var(--mono-font);color:var(--code)}kbd{color:var(--preformatted);border:1px solid var(--preformatted);border-bottom:3px solid var(--preformatted);border-radius:5px;padding:.1rem .4rem}pre{padding:1rem 1.4rem;max-width:100%;overflow:auto;color:var(--preformatted)}pre code{color:var(--preformatted);background:0 0;margin:0;padding:0}progress{width:100%}progress:indeterminate{background-color:var(--accent-bg)}progress::-webkit-progress-bar{border-radius:5px;background-color:var(--accent-bg)}progress::-webkit-progress-value{border-radius:5px;background-color:var(--accent)}progress::-moz-progress-bar{border-radius:5px;background-color:var(--accent);transition-property:width;transition-duration:.3s}progress:indeterminate::-moz-progress-bar{background-color:var(--accent-bg)}.notice{background:var(--accent-bg);border:2px solid var(--border);border-radius:5px;padding:1.5rem;margin:2rem 0;}";
const char PROGMEM kSTYLESHEET2[] =
    "hr { opacity: 0.3; }\n"
    "nav a.current {\n"
    "  color: var(--accent) !important;\n"
    "  border-color: var(--accent) !important;\n"
    "}\n"
    ".all-right td { text-align: right; }\n"
    "label.inline {\n"
    "  display: inline-block;\n"
    "  width: 160px;\n"
    "}\n"
    ".form-hint {\n"
    "  padding: 0 0 24px 172px;\n"
    "  font-size: 0.8em;\n"
    "}\n"
    ".secondary-button, .secondary-button:visited {\n"
    "  border: 1px solid var(--border);\n"
    "  background-color: var(--accent-bg);\n"
    "  color: var(--text) !important;\n"
    "  text-decoration: none !important;\n"
    "}\n"
    "thead td { font-weight: bold; }\n"
    "@media only screen and (max-width: 720px) {\n"
    "  .form-hint {\n"
    "    padding: 0 0 24px 0;\n"
    "  }\n"
    "  .secondary-button {\n"
    "    width: 100%;\n"
    "    text-align: center;\n"
    "  }\n"
    "}\n";

const char PROGMEM kREBOOTING[]  = "<p class=\"notice\">Rebooting, please wait...</p>\n";

const char* kBAUD       = "baud";
const char* kPWD        = "pwd";
const char* kSSID       = "ssid";
const char* kPWDSTA     = "pwdsta";
const char* kSSIDSTA    = "ssidsta";
const char* kIPSTA      = "ipsta";
const char* kGATESTA    = "gatewaysta";
const char* kSUBSTA     = "subnetsta";
const char* kCPORT      = "cport";
const char* kHPORT      = "hport";
const char* kCHANNEL    = "channel";
const char* kDEBUG      = "debug";
const char* kREBOOT     = "reboot";
const char* kPOSITION   = "position";
const char* kMODE       = "mode";

const char* kFlashMaps[7] = {
    "512KB (256/256)",
    "256KB",
    "1MB (512/512)",
    "2MB (512/512)",
    "4MB (512/512)",
    "2MB (1024/1024)",
    "4MB (1024/1024)"
};

// convert git version and build date to string
#define str(s) #s
#define xstr(s) str(s)
#define GIT_VERSION_STRING xstr(PIO_SRC_REV)
#define BUILD_DATE_STRING xstr(PIO_BUILD_DATE)
#define BOARD_STRING xstr(PIO_BOARD)

static uint32_t flash = 0;
static char paramCRC[12] = {""};

static void scheduleReboot(uint32_t delayMs);

WebServer           webServer(80);
MavESP8266Update*   updateCB    = NULL;
bool                started     = false;

//---------------------------------------------------------------------------------
void setNoCacheHeaders() {
    webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    webServer.sendHeader("Pragma", "no-cache");
    webServer.sendHeader("Expires", "0");
}

//---------------------------------------------------------------------------------
void setStaticFileCacheHeaders() {
    webServer.sendHeader("Cache-Control", "public, max-age=86400");
}

//---------------------------------------------------------------------------------
void returnFail(String msg) {
    webServer.send(500, FPSTR(kTEXTPLAIN), msg + "\r\n");
}

//---------------------------------------------------------------------------------
void respondOK() {
    webServer.send(200, FPSTR(kTEXTPLAIN), "OK");
}

//---------------------------------------------------------------------------------
void handle_update() {
    String message = FPSTR(kHEADER);

    message += FPSTR(kUPLOADFORM);
    message += FPSTR(kFOOTER);

    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
void handle_upload() {
    webServer.sendHeader("Connection", "close");
    webServer.sendHeader(FPSTR(kACCESSCTL), "*");
    webServer.send(200, FPSTR(kTEXTPLAIN), (Update.hasError()) ? "FAIL" : "OK");
    if(updateCB) {
        updateCB->updateCompleted();
    }
    scheduleReboot(500);
}

//---------------------------------------------------------------------------------
void handle_upload_status() {
    bool success  = true;
    if(!started) {
        started = true;
        if(updateCB) {
            updateCB->updateStarted();
        }
    }
    HTTPUpload& upload = webServer.upload();
    if(upload.status == UPLOAD_FILE_START) {
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.setDebugOutput(true);
        #endif

#if MAVESP8266_IS_ESP32
#else
        WiFiUDP::stopAll();
#endif

        Serial.end();
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.printf("Update: %s\n", upload.filename.c_str());
        #endif
        uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
        if(!Update.begin(maxSketchSpace)) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if(upload.status == UPLOAD_FILE_WRITE) {
        if(Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
    } else if (upload.status == UPLOAD_FILE_END) {
        if (Update.end(true)) {
            #ifdef DEBUG_SERIAL
                DEBUG_SERIAL.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
            #endif
        } else {
            #ifdef DEBUG_SERIAL
                Update.printError(DEBUG_SERIAL);
            #endif
            success = false;
        }
        #ifdef DEBUG_SERIAL
            DEBUG_SERIAL.setDebugOutput(false);
        #endif
    }
    yield();
    if(!success) {
        if(updateCB) {
            updateCB->updateError();
        }
    }
}

//---------------------------------------------------------------------------------
void handle_getParameters()
{
    String message = FPSTR(kHEADER);
    message += "<h2>Parameters</h2><table><thead><tr><td width=\"240\">Name</td><td>Value</td></tr></thead><tbody>";
    for(int i = 0; i < MavESP8266Parameters::ID_COUNT; i++) {
        message += "<tr><td><code>";
        message += getWorld()->getParameters()->getAt(i)->id;
        message += "</code></td>";
        unsigned long val = 0;
        if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT32)
            val = (unsigned long)*((uint32_t*)getWorld()->getParameters()->getAt(i)->value);
        else if(getWorld()->getParameters()->getAt(i)->type == MAV_PARAM_TYPE_UINT16)
            val = (unsigned long)*((uint16_t*)getWorld()->getParameters()->getAt(i)->value);
        else
            val = (unsigned long)*((int8_t*)getWorld()->getParameters()->getAt(i)->value);
        message += "<td>";
        message += val;
        message += "</td></tr>";
    }
    message += "</tbody></table>";
    message += FPSTR(kFOOTER);
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_root()
{
    String message = FPSTR(kHEADER);
    message += "<dl><dt>\n";
    message += "Version</dt><dd>";
    char vstr[30];
    snprintf(vstr, sizeof(vstr), "%u.%u.%u", MAVESP8266_VERSION_MAJOR, MAVESP8266_VERSION_MINOR, MAVESP8266_VERSION_BUILD);
    message += vstr;
    message += "</dd><dt>\n";
    message += "Git Version</dt><dd>";
    message += GIT_VERSION_STRING;
    message += "</dd><dt>\n";
    message += "Build Date</dt><dd>";
    message += BUILD_DATE_STRING;
    message += "</dd><dt>\n";
    message += "Board</dt><dd>";
    message += BOARD_STRING;
    message += "</dd><dt>\n";
    message += "WiFi MAC Address</dt><dd>";
    message += WiFi.macAddress();
    message += "</dd></dl>\n";
    message += "<h2>Documentation</h2>\n";
    message += "<p>(requires internet access)</p>\n";
    message += "<ul>\n";
    message += "<li><a href='http://ardupilot.org'>ArduPilot Website</a>\n";
    message += "<li><a href='http://skybrush.io'>Skybrush Website</a>\n";
    message += "<li><a href='http://ardupilot.org/copter/docs/common-esp8266-telemetry.html'>ESP8266 WiFi Documentation</a>\n";
    message += "<li><a href='https://github.com/skybrush-io/mavesp8266'>ESP8266 Source Code</a>\n";
    message += "</ul>\n";
    message += FPSTR(kFOOTER);
    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_setup()
{
    String message = FPSTR(kHEADER);
    message += "<h2>Setup</h2>\n";

    message += "<form action='/setparameters' method='post'>\n";

    message += "<div>\n";
    message += "<label class='inline' for='field-mode'>WiFi Mode</label>\n";
    message += "<select id='field-mode' name='mode' onchange='updateFieldVisibility(event.target.value)'>\n";
    message += "<option value='0'";
    if (getWorld()->getParameters()->getWifiMode() == MAVESP8266_WIFI_MODE_AP) {
        message += " selected";
    }
    message += ">Access Point</option>\n";
    message += "<option value='1'";
    if (getWorld()->getParameters()->getWifiMode() == MAVESP8266_WIFI_MODE_CLIENT) {
        message += " selected";
    }
    message += ">Client</option>\n";
    message += "</select>\n";
    message += "<div class='form-hint only-for-ap'>AP mode creates an AP on its own</div>\n";
    message += "<div class='form-hint only-for-sta'>Client mode connects to an existing AP</div>\n";
    message += "</div>\n";
    
    message += "<div class='only-for-ap'>\n";
    message += "<label class='inline' for='field-ssid'>AP SSID</label>\n";
    message += "<input id='field-ssid' type='text' name='ssid' value='";
    message += getWorld()->getParameters()->getWifiSsid();
    message += "' />\n";
    message += "</div>\n";

    message += "<div class='only-for-ap'>\n";
    message += "<label class='inline' for='field-pwd'>AP Password</label>\n";
    message += "<input id='field-pwd' type='text' name='pwd' value='";
    message += getWorld()->getParameters()->getWifiPassword();
    message += "' />\n";
    message += "<div class='form-hint'>Minimum length: 8 characters</div>\n";
    message += "</div>\n";

    message += "<div class='only-for-ap'>\n";
    message += "<label class='inline' for='field-channel'>WiFi Channel</label>\n";
    message += "<input id='field-channel' type='text' name='channel' value='";
    message += getWorld()->getParameters()->getWifiChannel();
    message += "' />";
    message += "</div>\n";

    message += "<div class='only-for-sta'>\n";
    message += "<label class='inline' for='field-ssidsta'>Station SSID</label>\n";
    message += "<input id='field-ssidsta' type='text' name='ssidsta' value='";
    message += getWorld()->getParameters()->getWifiStaSsid();
    message += "' />";
    message += "</div>\n";

    message += "<div class='only-for-sta'>\n";
    message += "<label class='inline' for='field-pwdsta'>Station Password</label>\n";
    message += "<input id='field-pwdsta' type='text' name='pwdsta' value='";
    message += getWorld()->getParameters()->getWifiStaPassword();
    message += "' />\n";
    message += "</div>\n";

    IPAddress IP;
    message += "<div class='only-for-sta'>\n";
    message += "<label class='inline' for='field-ipsta'>IP Address</label>\n";
    message += "<input id='field-ipsta' type='text' name='ipsta' value='";
    IP = getWorld()->getParameters()->getWifiStaIP();
    message += IP.toString();
    message += "' />";
    message += "</div>\n";

    message += "<div class='only-for-sta'>\n";
    message += "<label class='inline' for='field-gatewaysta'>Gateway</label>\n";
    message += "<input id='field-gatewaysta' type='text' name='gatewaysta' value='";
    IP = getWorld()->getParameters()->getWifiStaGateway();
    message += IP.toString();
    message += "' />";
    message += "</div>\n";

    message += "<div class='only-for-sta'>\n";
    message += "<label class='inline' for='field-subnetsta'>Subnet Mask</label>\n";
    message += "<input id='field-subnetsta' type='text' name='subnetsta' value='";
    IP = getWorld()->getParameters()->getWifiStaSubnet();
    message += IP.toString();
    message += "' />";
    message += "</div>\n";

    message += "<hr />\n";

    message += "<div>\n";
    message += "<label class='inline' for='field-hport'>GCS UDP Port</label>\n";
    message += "<input id='field-hport' type='text' name='hport' value='";
    message += getWorld()->getParameters()->getWifiUdpHport();
    message += "' />";
    message += "</div>\n";

    message += "<div>\n";
    message += "<label class='inline' for='field-cport'>Vehicle UDP Port</label>\n";
    message += "<input id='field-cport' type='text' name='cport' value='";
    message += getWorld()->getParameters()->getWifiUdpCport();
    message += "' />";
    message += "</div>\n";
    
    message += "<hr />\n";

    message += "<div>\n";
    message += "<label class='inline' for='field-baud'>UART Baudrate</label>\n";
    message += "<input id='field-baud' type='text' name='baud' value='";
    message += getWorld()->getParameters()->getUartBaudRate();
    message += "' />";
    message += "</div>\n";
    
    message += "<input type='submit' value='Save Settings' />\n";
    message += "<a class='secondary-button' role='button' href='/setdefaults' style='float: right; line-height: 1' onclick=\"return confirm('Are you sure?')\">Reset to Defaults</a>\n";
    message += "</form>\n";

    message += "<script>\n";
    message += "function updateFieldVisibility(value) {\n";
    message += "  value = String(value);\n";
    message += "  let elts = document.getElementsByClassName('only-for-ap');\n";
    message += "  Array.prototype.map.call(elts, (elt) => elt.style.display = value == '0' ? 'block' : 'none');\n";
    message += "  elts = document.getElementsByClassName('only-for-sta');\n";
    message += "  Array.prototype.map.call(elts, (elt) => elt.style.display = value == '1' ? 'block' : 'none');\n";
    message += "};\n";
    message += "window.addEventListener('DOMContentLoaded', function() { updateFieldVisibility(document.getElementById('field-mode').value); })\n";
    message += "</script>\n";

    message += FPSTR(kFOOTER);

    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}


//---------------------------------------------------------------------------------
static void handle_getStatus()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    String message = FPSTR(kHEADER);
    message += "<h2>Comm Status</h2>\n";
    message += "<table class=\"all-right\">\n";
    message += "<thead><tr><td>&nbsp;</td><td>Address</td><td>Received</td><td>Sent</td><td>Lost</td><td>Error</td></tr></thead>\n";
    message += "<tbody><tr><td>GCS</td><td>";
    message += getWorld()->getGCS()->getIPAddress().toString();
    message += "</td><td>";
    message += gcsStatus->packets_received;
    message += "</td><td>";
    message += gcsStatus->packets_sent;
    message += "</td><td>";
    message += gcsStatus->packets_lost;
    message += "</td><td>";
    message += gcsStatus->parse_errors;
    message += "</td></tr><tr><td>Vehicle</td><td>";
    message += getWorld()->getVehicle()->getIPAddress().toString();
    message += "</td><td>";
    message += vehicleStatus->packets_received;
    message += "</td><td>";
    message += vehicleStatus->packets_sent;
    message += "</td><td>";
    message += vehicleStatus->packets_lost;
    message += "</td><td>";
    message += vehicleStatus->parse_errors;
    message += "</td></tr><tr><td>Radio status</td><td>";
    message += "&nbsp;";
    message += "</td><td>";
    message += "&nbsp;";
    message += "</td><td>";
    message += gcsStatus->radio_status_sent;
    message += "</td><td>";
    message += "&nbsp;";
    message += "</td><td>";
    message += "&nbsp;";
    message += "</td></tr></tbody></table>";
    message += "<h2>System Status</h2><table><tbody>\n";
    message += "<tr><td width=\"420\">Flash Size</td><td>";
#if MAVESP8266_IS_ESP32
    message += String(spi_flash_get_chip_size());
#else
    message += String(ESP.getFlashChipRealSize());
#endif
    message += "</td></tr>\n";
    message += "<tr><td>Flash Available</td><td>";
    message += flash;
    message += "</td></tr>\n";
    message += "<tr><td>RAM Left</td><td>";
    message += String(ESP.getFreeHeap());
    message += "</td></tr>\n";
    message += "<tr><td>Parameters CRC</td><td>";
    message += paramCRC;
    message += "</td></tr>\n";
    message += "<tr><td>Raw mode</td><td>";
    message += getWorld()->getComponent()->inRawMode() ? "on" : "off";

    if (getWorld()->getPowerMgmt()->supportsReadingPowerState()) {
        message += "</td></tr>\n";
        message += "<tr><td>Vehicle power</td><td>";
        message += getWorld()->getPowerMgmt()->isPowerOn() ? "on" : "off";
    }

    message += "</td></tr>\n";
    message += "</tbody></table>";
    message += FPSTR(kFOOTER);

    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);
}

//---------------------------------------------------------------------------------
static void handle_getStylesheet()
{
    setStaticFileCacheHeaders();
    webServer.send(200, FPSTR(kTEXTCSS), FPSTR(kSTYLESHEET));
}

//---------------------------------------------------------------------------------
static void handle_getStylesheet_Custom()
{
    setStaticFileCacheHeaders();
    webServer.send(200, FPSTR(kTEXTCSS), FPSTR(kSTYLESHEET2));
}

//---------------------------------------------------------------------------------
void handle_getJLog()
{
    uint32_t position = 0, len;
    if(webServer.hasArg(kPOSITION)) {
        position = webServer.arg(kPOSITION).toInt();
    }
    String logText = getWorld()->getLogger()->getLog(&position, &len);
    char jStart[128];
    snprintf(jStart, 128, "{\"len\":%d, \"start\":%d, \"text\": \"", len, position);
    String payLoad = jStart;
    payLoad += logText;
    payLoad += "\"}";
    webServer.send(200, FPSTR(kAPPJSON), payLoad);
}

//---------------------------------------------------------------------------------
void handle_getJSysInfo()
{
    if(!flash)
        flash = ESP.getFreeSketchSpace();
    if(!paramCRC[0]) {
        snprintf(paramCRC, sizeof(paramCRC), "%08X", getWorld()->getParameters()->paramHashCheck());
    }
#ifdef MAVESP8266_IS_ESP32
    uint32_t fid = 0;  /* TODO */
#else
    uint32_t fid = spi_flash_get_id();
#endif
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"size\": \"%s\", "
        "\"id\": \"0x%02lX 0x%04lX\", "
        "\"flashfree\": \"%u\", "
        "\"heapfree\": \"%u\", "
        "\"logsize\": \"%u\", "
        "\"paramcrc\": \"%s\""
        " }",
#ifdef MAVESP8266_IS_ESP32
        ESP.getFlashChipSize(),
#else
        kFlashMaps[system_get_flash_size_map()],
#endif
        (long unsigned int)(fid & 0xff), (long unsigned int)((fid & 0xff00) | ((fid >> 16) & 0xff)),
        flash,
        ESP.getFreeHeap(),
        getWorld()->getLogger()->getPosition(),
        paramCRC
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_getJSysStatus()
{
    bool reset = false;
    if(webServer.hasArg("r")) {
        reset = webServer.arg("r").toInt() != 0;
    }
    linkStatus* gcsStatus = getWorld()->getGCS()->getStatus();
    linkStatus* vehicleStatus = getWorld()->getVehicle()->getStatus();
    if(reset) {
        memset(gcsStatus,     0, sizeof(linkStatus));
        memset(vehicleStatus, 0, sizeof(linkStatus));
    }
    char message[512];
    snprintf(message, 512,
        "{ "
        "\"gpackets\": \"%u\", "
        "\"gsent\": \"%u\", "
        "\"glost\": \"%u\", "
        "\"vpackets\": \"%u\", "
        "\"vsent\": \"%u\", "
        "\"vlost\": \"%u\", "
        "\"radio\": \"%u\", "
        "\"buffer\": \"%u\""
        " }",
        gcsStatus->packets_received,
        gcsStatus->packets_sent,
        gcsStatus->packets_lost,
        vehicleStatus->packets_received,
        vehicleStatus->packets_sent,
        vehicleStatus->packets_lost,
        gcsStatus->radio_status_sent,
        vehicleStatus->queue_status
    );
    webServer.send(200, "application/json", message);
}

//---------------------------------------------------------------------------------
void handle_setParameters()
{
    if(webServer.args() == 0) {
        returnFail(kBADARG);
        return;
    }
    bool ok = false;
    bool reboot = false;
    if(webServer.hasArg(kBAUD)) {
        ok = true;
        getWorld()->getParameters()->setUartBaudRate(webServer.arg(kBAUD).toInt());
    }
    if(webServer.hasArg(kPWD)) {
        ok = true;
        getWorld()->getParameters()->setWifiPassword(webServer.arg(kPWD).c_str());
    }
    if(webServer.hasArg(kSSID)) {
        ok = true;
        getWorld()->getParameters()->setWifiSsid(webServer.arg(kSSID).c_str());
    }
    if(webServer.hasArg(kPWDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaPassword(webServer.arg(kPWDSTA).c_str());
    }
    if(webServer.hasArg(kSSIDSTA)) {
        ok = true;
        getWorld()->getParameters()->setWifiStaSsid(webServer.arg(kSSIDSTA).c_str());
    }
    if(webServer.hasArg(kIPSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kIPSTA).c_str());
        getWorld()->getParameters()->setWifiStaIP(ip);
        ok = true;
    }
    if(webServer.hasArg(kGATESTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kGATESTA).c_str());
        getWorld()->getParameters()->setWifiStaGateway(ip);
        ok = true;
    }
    if(webServer.hasArg(kSUBSTA)) {
        IPAddress ip;
        ip.fromString(webServer.arg(kSUBSTA).c_str());
        getWorld()->getParameters()->setWifiStaSubnet(ip);
        ok = true;
    }
    if(webServer.hasArg(kCPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpCport(webServer.arg(kCPORT).toInt());
    }
    if(webServer.hasArg(kHPORT)) {
        ok = true;
        getWorld()->getParameters()->setWifiUdpHport(webServer.arg(kHPORT).toInt());
    }
    if(webServer.hasArg(kCHANNEL)) {
        ok = true;
        getWorld()->getParameters()->setWifiChannel(webServer.arg(kCHANNEL).toInt());
    }
    if(webServer.hasArg(kDEBUG)) {
        ok = true;
        getWorld()->getParameters()->setDebugEnabled(webServer.arg(kDEBUG).toInt());
    }
    if(webServer.hasArg(kMODE)) {
        ok = true;
        getWorld()->getParameters()->setWifiMode(webServer.arg(kMODE).toInt());
    }
    if(webServer.hasArg(kREBOOT)) {
        ok = true;
        reboot = webServer.arg(kREBOOT) == "1";
    }
    if(ok) {
        getWorld()->getParameters()->saveAllToEeprom();
        //-- Send new parameters back
        handle_getParameters();
        if(reboot) {
            scheduleReboot(100);
        }
    } else
        returnFail(kBADARG);
}

//---------------------------------------------------------------------------------
void handle_rawMode()
{
    if(webServer.args() == 0) {
        handle_getStatus();
        return;
    }

    if(webServer.hasArg(kMODE)) {
        if (webServer.arg(kMODE).toInt()) {
            getWorld()->getComponent()->enableRawMode();
        } else {
            getWorld()->getComponent()->disableRawMode();
        }
        handle_getStatus();
    } else
        returnFail(kBADARG);
}

//---------------------------------------------------------------------------------
static void handle_reboot()
{
    String message = FPSTR(kHEADER);
    message += FPSTR(kREBOOTING);
    message += FPSTR(kFOOTER);

    setNoCacheHeaders();
    webServer.send(200, FPSTR(kTEXTHTML), message);

    scheduleReboot(500);
}

//---------------------------------------------------------------------------------
void handle_setDefaults()
{
    getWorld()->getParameters()->resetToDefaults();
    getWorld()->getParameters()->saveAllToEeprom();
    handle_setup();
}

//---------------------------------------------------------------------------------
//-- 404
void handle_notFound(){
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += webServer.uri();
    message += "\nMethod: ";
    message += (webServer.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += webServer.args();
    message += "\n";
    for (uint8_t i = 0; i < webServer.args(); i++){
        message += " " + webServer.argName(i) + ": " + webServer.arg(i) + "\n";
    }
    webServer.send(404, FPSTR(kTEXTPLAIN), message);
}

//---------------------------------------------------------------------------------
MavESP8266Httpd::MavESP8266Httpd()
{

}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Httpd::begin(MavESP8266Update* updateCB_)
{
    updateCB = updateCB_;
    webServer.on("/",               handle_root);
    webServer.on("/getparameters",  handle_getParameters);
    webServer.on("/setparameters",  handle_setParameters);
    webServer.on("/getstatus",      handle_getStatus);
    webServer.on("/raw",            handle_rawMode);
    webServer.on("/reboot",         handle_reboot);
    webServer.on("/setup",          handle_setup);
    webServer.on("/info.json",      handle_getJSysInfo);
    webServer.on("/status.json",    handle_getJSysStatus);
    webServer.on("/log.json",       handle_getJLog);
    webServer.on("/style.css",      handle_getStylesheet);
    webServer.on("/custom.css",     handle_getStylesheet_Custom);
    webServer.on("/update",         handle_update);
    webServer.on("/upload",         HTTP_POST, handle_upload, handle_upload_status);
    webServer.on("/setdefaults",    handle_setDefaults);
    webServer.onNotFound(           handle_notFound);
    webServer.begin();
}

//---------------------------------------------------------------------------------
//-- Initialize
void
MavESP8266Httpd::checkUpdates()
{
    webServer.handleClient();
}

//---------------------------------------------------------------------------------
//-- Schedule reboot on main loop
static void scheduleReboot(uint32_t delayMs)
{
    if (updateCB) {
        updateCB->scheduleReboot(delayMs);
    } else {
        delay(delayMs);
        ESP.restart();
    }
}
