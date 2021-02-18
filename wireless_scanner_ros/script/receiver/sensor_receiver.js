#!/usr/bin/env node

// Copyright (c) 2021  IBM Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// http web server
var http = require('http');
var fs = require('fs');
var querystring = require('querystring');
// ros bridge
var roslib = require('roslib');
var ros = new roslib.Ros({
  url : 'ws://localhost:9090'
});

require('date-utils');
var sleep = require('sleep');
sleep.sleep(1)

// ros setting
ros.on('error', function(err){
  console.log('Error on connection to the ros bridge server.');
  console.log(err);
});
ros.on('connection', function() {
  console.log('Connected to the ros bridge server.');
});

var beaconsTopic = new roslib.Topic({
  ros: ros,
  name: '/beacons',
  messageType: '/std_msgs/String'
})

var accessibilityTopic = new roslib.Topic({
  ros: ros,
  name: '/accessibility',
  messageType: '/std_msgs/String'
})

var generalSensorTopic = new roslib.Topic({
  ros: ros,
  name: '/mobile_sensor',
  messageType: '/std_msgs/String'
})

var locationTopic = new roslib.Topic({
  ros: ros,
  name: '/mobile_location',
  messageType: '/std_msgs/String'
})

var wifiScanStrTopic = new roslib.Topic({
  ros: ros,
  name: '/wireless/wifi_iwlist_scan_str',
  messageType: '/std_msgs/String'
})

var wifiIwScanStrTopic = new roslib.Topic({
  ros: ros,
  name: '/wireless/wifi_iw_scan_str',
  messageType: '/std_msgs/String'
})

var beaconScanStrTopic = new roslib.Topic({
  ros: ros,
  name: '/wireless/beacon_scan_str',
  messageType: '/std_msgs/String'
})

// server setting
var server = http.createServer(function (req, res) {
  // Set CORS headers
  res.setHeader('Access-Control-Allow-Origin', '*');
  res.setHeader('Access-Control-Request-Method', '*');
  res.setHeader('Access-Control-Allow-Methods', 'OPTIONS, GET, POST');
  res.setHeader('Access-Control-Allow-Headers', '*');

  // default
  if(req.url == '/' && req.method == 'GET'){
    fs.readFile(__dirname + '/index.html', {
        encoding:'utf8'
    }, function(err, html){
       if(err){
          res.statusCode = 500;
          res.end('Error!');
       }else{
          res.setHeader('Content-Type', 'text/html');
          res.end(html);
       }
    }
    )
  }

  // beacon post request
  if(req.url == '/beacon' && req.method == 'POST'){
    var body = '';
    req.on('data', function(chunk) {
      body += chunk;
    }).on('end', function() {
      var dt = new Date();
      var formatted = dt.toFormat("YYYYMMDDHH24MISS");
      // console.log('beacon POST at ' + formatted);
      var post = querystring.parse(body); // not used
            var beaconsMessage = new roslib.Message({
          data : body
      });
      beaconsTopic.publish(beaconsMessage);
      // just for console cout
      var obj = JSON.parse(body);
      var data = obj.data;
      var str = "";
      if(data.length==0){
          str = "count="+data.length;
      }else{
          str = "count="+data.length + ",type=" + data[0].type;
      }
      console.log('beacon POST: time=' + formatted + ","+str + ",phone=" +obj.phoneID);
      res.end('OK');
    });
  }

  // location post
  if(req.url == '/mobile_location' && req.method == 'POST'){
    var body = '';
    req.on('data', function(chunk) {
      body += chunk;
    }).on('end', function() {
      var dt = new Date();
      var formatted = dt.toFormat("YYYYMMDDHH24MISS");
      // console.log('beacon POST at ' + formatted);
      var post = querystring.parse(body); // not used
            var locationMessage = new roslib.Message({
          data : body
      });
      locationTopic.publish(locationMessage);
      // just for console cout
      var obj = JSON.parse(body);
      var data = obj.data;
      var str = "count="+data.length;
      if(data.length>0){""
          str = str + ",type=" + data[0].type + ",latitude=" + data[0].latitude + ",longitude=" + data[0].longitude;
      }
      console.log('mobile_location POST: time=' + formatted + ","+str + ",phone=" +obj.phoneID);
      res.end('OK');
    });
  }

  // accessibility post request
  if(req.url == '/accessibility' && req.method == 'POST'){
    var dt = new Date();
    var formatted = dt.toFormat("YYYYMMDDHH24MISS");
    console.log('accessibility POST at ' + formatted);
    var body = '';
    req.on('data', function(chunk) {
      body += chunk;
    }).on('end', function() {
      var post = querystring.parse(body);
      var accessibilityMessage = new roslib.Message({
          data : body
      });
      accessibilityTopic.publish(accessibilityMessage);
      res.end('OK');
    });
  }

  // mobile_sensor post request
  if(req.url == '/mobile_sensor' && req.method == 'POST'){
    var dt = new Date();
    var formatted = dt.toFormat("YYYYMMDDHH24MISS");
    var body = '';
    req.on('data', function(chunk) {
      body += chunk;
    }).on('end', function() {
      var post = querystring.parse(body);
      var message = new roslib.Message({
          data : body
      });
      generalSensorTopic.publish(message);
      console.log('mobile_sensor POST at ' + formatted + ": " + body);
      res.end('OK');
    });
  }

  // wifi_scan_str (wifi_iwlist_scan_str)
  if(req.url == '/wifi_iwlist_scan_str' && req.method == 'POST'){
    var dt = new Date();
    var formatted = dt.toFormat("YYYYMMDDHH24MISS");
    console.log('wifi_iwlist_scan_str POST at ' + formatted);
    var body = '';
    req.on('data', function(chunk) {
      body += chunk;
    }).on('end', function() {
      var post = querystring.parse(body);
      var message = new roslib.Message({
          data : body
      });
      wifiScanStrTopic.publish(message);
      res.end('OK');
    });
  }

  // wifi_iw_scan_str
  if(req.url == '/wifi_iw_scan_str' && req.method == 'POST'){
    var dt = new Date();
    var formatted = dt.toFormat("YYYYMMDDHH24MISS");
    console.log('wifi_scan_str POST at ' + formatted);
    var body = '';
    req.on('data', function(chunk) {
      body += chunk;
    }).on('end', function() {
      var post = querystring.parse(body);
      var message = new roslib.Message({
          data : body
      });
      wifiIwScanStrTopic.publish(message);
      res.end('OK');
    });
  }

  // beacon_scan_str
  if(req.url == '/beacon_scan_str' && req.method == 'POST'){
    var dt = new Date();
    var formatted = dt.toFormat("YYYYMMDDHH24MISS");
    console.log('beacon_scan_str POST at ' + formatted);
    var body = '';
    req.on('data', function(chunk) {
      body += chunk;
    }).on('end', function() {
      var post = querystring.parse(body);
      var message = new roslib.Message({
          data : body
      });
      beaconScanStrTopic.publish(message);
      res.end('OK');
    });
  }
});

server.listen(8080, '0.0.0.0');
