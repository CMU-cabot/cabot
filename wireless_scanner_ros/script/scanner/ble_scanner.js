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

// functions for lock
var fs = require('fs');
var path = require('path');
var fname = path.basename(process.argv[1])

const lockFilePath = "/var/run/"+fname+".lock";
function lockFile() {
    try {
        var file = fs.openSync(lockFilePath, 'r');
        fs.close(file);
        return false;
    } catch (e1) {
        try {
            fs.writeFileSync(lockFilePath);
            return true;
        } catch (e2) {
            console.error(e2);
            return false;
        }
    }
}

function unlockFile() {
    fs.unlink(lockFilePath, function(err){
        if (err) throw err;
    })
}

// main

var argv = require('argv')
var request = require('request');
timeout = 100; // ms

const BeaconScanner = require('node-beacon-scanner');
const scanner = new BeaconScanner();

argv.option([
    {
	name: 'quiet',
	short: 'q',
	type: 'bool',
	description: 'suppress output',
	example: "./ble_scanner.js -q"
    },
    {
  name: 'no_lock',
  short: 'n',
  type: 'bool',
  description: 'disable lock/unlock for special use cases',
  example: "./ble_scanner.js -n"
    }
  ]
)

args = argv.run();
quiet = args.options.quiet || false
no_lock = args.options.no_lock || false

// lock
if (!no_lock){
  if(!lockFile()){
      console.error("Another "+fname+" proccess is running. abort");
      process.exit();
  }

  process.on('SIGINT', function(){
    unlockFile();
    console.log( fname + " is closed.")
    process.exit();
  });
  process.on('SIGTERM', function(){
    unlockFile();
    console.log( fname + " is closed.")
    process.exit();
  });
}

// Set an Event handler for beacons
scanner.onadvertisement = (ad) => {
  var options = {
      url: 'http://localhost:8080/beacon_scan_str',
      method: 'POST',
      json: ad,
      timeout: timeout
  }
    if (ad) {
	if (!quiet) {
	    console.log(JSON.stringify(ad))
	}
    }
  request(options, function (error, response, body){
    if(error){
	console.error(JSON.stringify(error))
    } else {
	if (!quiet) {
	    console.log("body= "+body);
	}
    }
  })
};

// Start scanning
scanner.startScan().then(() => {
  console.log('Started to scan.')  ;
}).catch((error) => {
  console.error(error);
});
