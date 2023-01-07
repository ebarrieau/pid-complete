let clamp = function(val, low, high) {
  if (val > high) return high;
  if (val < low) return low;
  return val;
}


// Symbols for autotune controller state
const START = Symbol();
const UP = Symbol();
const DOWN = Symbol();
const FINISH = Symbol();
const OFF = Symbol();

const _tuning_rules = {
    "pid": [0.6, 0.5, .125],
    "pi": [0.45, 0.8333, 0],
    "pd": [0.8, 0, 0.125],
    "pessen-integral": [0.7, 0.4, 0.15],
    "some-overshoot": [0.3333, 0.50,  0.3333],
    "no-overshoot": [0.20, 0.50, 0.3333],
    }


module.exports = function(RED) {
    function CompletePIDAutotuneNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;

        node.name = config.name || "PID Autotune";
        node.outDec = parseFloat(config.outDec) || 0;
        node.outInc = parseFloat(config.outInc) || 1;
        node.disabledOut = parseFloat(config.disabledOut) || 0;
        node.rule = config.tuning || "ziegler-nichols";
        node.after = config.after || "disabled";
        node.stepPercent = parseFloat(config.stepPercent) || 1;

        node.state = OFF;

        function startAutotune() {
          let autoTuneHysteresis = node.sp * node.stepPercent / 100;
          let now = Date.now();
          node._autoTuneLow = node.sp - autoTuneHysteresis;
          node._autoTuneHigh = node.sp + autoTuneHysteresis;
          node._Peaks = [];
          node._MinPeak = {value: node.sp, time: now};
          node._MaxPeak = {value: node.sp, time: now};
          node.state = START;
          node.status({fill:"green", text:"Autotuning"});
        }

        function stopAutotune(success = false) {
          node.state = OFF;
          if (node.state != OFF) {
            if (success) {
              node.status({fill: "grey", text:"Success"});
            } else {
              node.status({fill: "red", text:"Stopped"});
            }
          } else {
            node.status({});
          }
        }

        function autoTune(input) {
          let now = Date.now();
          let ret = false;
          if (input > node._MaxPeak.value) {
            node._MaxPeak = {value: input, time: now};
          }

          if (input < node._MinPeak.value) {
            node._MinPeak = {value: input, time: now};
          }
          switch (node.state) {
            case START:
              node.output = node.outDec;
              if (input < node._autoTuneLow) {
                node.state = UP;
                node.output = node.outInc;
              }
              break;

            case UP:
              //Ignore the first peak, before there is no min recorded on the
              //first ramp up
              if (input > node._autoTuneHigh) {
                if (node._Peaks.length >= 1) {
                  node._Peaks.push(node._MinPeak);
                }

                node._MinPeak = {value: input, time: now};

                if (node._Peaks.length >= 4) {
                  node.output = node.outDec;
                  node.state = FINISH;
                } else {
                  node.state = DOWN;
                  node.output = node.outDec;
                }
              }
              break;

            case DOWN:

              if (input < node._autoTuneLow) {
                node._Peaks.push(node._MaxPeak);
                node._MaxPeak = {value: input, time: now};
                node.output = node.outInc;
                node.state = UP;
              }
              break;

            case FINISH:
              // Peak 0 = First High Peak
              // Peak 1 = First Low Peak
              // Peak 2 = Second High Peak
              // Peak 3 = Second Low Peak

              //Calculate the actual difference of the high and low peak
              //and take the average of both sets
              let amplitude1 = node._Peaks[0].value - node._Peaks[1].value;
              let amplitude2 = node._Peaks[2].value - node._Peaks[3].value;

              let amplitude = (amplitude1 + amplitude2) / 2;

              //Ku is 4/PI * the ratio of commanded differece vs actual difference
              node._Ku = 4.0 * (node._autoTuneHigh - node._autoTuneLow) / (amplitude * Math.PI);


              //Calculate the actual period from High Peak to High Peak and
              //Low Peak to Low Peak and take the average.
              let period1 = node._Peaks[2].time - node._Peaks[0].time;
              let period2 = node._Peaks[3].time - node._Peaks[1].time;

              let period = (period1 + period2) / 2;

              //Date.now() returns milliseconds, divide by 1000 to get period in seconds
              node._Pu = period / 1000.0;

              console.log("Ku: " + node._Ku + " Pu: " + node._Pu);

              ret = true;

              break;
          }
          return ret;
        }

        node.on('input', function(msg, send, done) {
          if (msg.hasOwnProperty("cmd")) {
            switch (msg.cmd) {
              case "disable":
                node.prevState = "disable";
                stopAutotune();
                break;
              case "manual":
                node.prevState = "manual";
                if (msg.hasOwnProperty("payload")) {
                  node.prevManual = parseFloat(msg.payload);
                } else {
                  node.prevManual = node.disabledOut
                }
                stopAutotune();
                break;
              case "auto":
                node.prevState = "auto";
                stopAutotune();
                break;
              case "autotune":
                startAutotune();
                break;
            }
          }  else if (msg.hasOwnProperty("topic")) {
            if (msg.hasOwnProperty("payload")) {
              switch (msg.topic) {
                case "setpoint":
                case "sp":
                  node.sp = parseFloat(msg.payload);
                  break;
                }
              }
            } else if (msg.hasOwnProperty("payload")) {
              if (node.state != OFF) {
                let tempOutput = parseFloat(msg.payload);
                if (!isNaN(tempOutput)) {
                  let ret = autoTune(tempOutput);
                  if (ret) {
                    node.state = OFF;
                    let divisors = _tuning_rules[node.rule];
                    node.kp = node._Ku / divisors[0];
                    node.ki = node.kp / (node._Pu * divisors[1]);
                    node.kd = node.kp * (node._Pu * divisors[2]);
                    let tunings = [{topic: "kp", payload: node.kp},
                                   {topic: "ki", payload: node.ki},
                                   {topic: "kd", payload: node.kd}];
                    tunings.forEach( (tuning) => {
                      send(tuning);
                    });

                    console.log(node.kp, node.ki, node.kd);

                    let newMsg = {};
                    switch (node.after) {
                      case "manual":
                      case "disable":
                      case "auto":
                        newMsg.cmd = node.after;
                        break;
                      case "previous":
                        newMsg.cmd = node.prevState;
                        break;
                    }
                    if (newMsg.cmd === "manual") newMsg.payload = node.prevManual || node.disabledOut;

                    send(newMsg);
                  } else {
                    let newMsg = {cmd: "manual", payload: node.output};
                    send(newMsg);
                  }
                }
              }
            }

          send(msg);

          if (done) done();
        });
    }
    RED.nodes.registerType("pid-complete-autotune",CompletePIDAutotuneNode);
}
