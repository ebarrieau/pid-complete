let clamp = function(val, low, high) {
  if (val > high) return high;
  if (val < low) return low;
  return val;
}

const IDLE = Symbol();
const UP = Symbol();
const DOWN = Symbol();
const FINISH = Symbol();
const OFF = Symbol();

const _tuning_rules = {
    "ziegler-nichols": [34, 40, 160],
    "tyreus-luyben": [44,  9, 126],
    "ciancone-marlin": [66, 88, 162],
    "pessen-integral": [28, 50, 133],
    "some-overshoot": [60, 40,  60],
    "no-overshoot": [100, 40,  60],
    "brewing": [2.5, 6, 380]
    }

module.exports = function(RED) {
    function CompletePIDNode(config) {
        RED.nodes.createNode(this,config);
        var node = this;

        node.sampleSec = parseFloat(config.sampleInterval) || 1;
        node.outMin = parseFloat(config.outMin) || 0;
        node.outMax = parseFloat(config.outMax) || 1;
        node.kp = parseFloat(config.kp) || 1;
        node.ki = parseFloat(config.ki) || 0.01;
        node.kd = parseFloat(config.kd) || 0;
        node.sp = parseFloat(config.sp) || 100;
        node.PonM = config.PonM || false;
        node.DonM = config.DonM || false;
        node.bumpPercent = parseFloat(config.bumpPercent) || 1;
        node.tuningRule = config.tuning || "ziegler-nichols";

        node.enabled = false;
        node.auto = false;
        node._autoTuneState = OFF;

        function reset() {
          node._proportional = 0;
          node._integral = 0;
          node._derivative = 0;

          node._lastTime = Date.now();
          node._lastInput = null;
          node._lastOutput = null;
          node._lastError = null;
        }

        function setAuto(output = null) {
          if (!node.enabled) return;

          if (!node.auto) {
            reset();
            node._integral = output ||  0;
            node._integral = clamp(node._integral, node.outMin, node.outMax);
            node.auto = true;
          }
        }

        function setManual(output) {
          if (!node.enabled) return;

          reset();
          node._lastOutput = output || node._lastOutput;
          node._lastOutput = clamp(node._lastOutput, node.outMin, node.outMax);
          node.auto = false;

        }

        function startAutotune() {
          reset();
          let autoTuneCenter = node.sp;
          let autoTuneHysteresis = node.sp * node.bumpPercent / 100;
          node._autoTuneLow = autoTuneCenter - autoTuneHysteresis;
          node._autoTuneHigh = autoTuneCenter + autoTuneHysteresis;
          node._autoTunePeaks = [];
          node._autoTuneMinPeak = [0, 0];
          node._autoTuneMaxPeak = [0, 0];
          node._autoTuneState = IDLE;
        }

        function calculate(input) {
          let now = Date.now();
          let dt = (now - (node._lastTime || 1e-16)) / 1000;
          if (dt < node.sampleSec) return node._lastOutput;

          let error = node.sp - input;
          let dInput = input - (node._lastInput || input);
          let dError = error - (node._lastError || error);

          if (node.PonM) {
            node._proportional -= node.kp * dInput;
          } else {
            node._proportional = node.kp * error;
          }

          node._integral += node.ki * error * dt;
          node._integral = clamp(node._integral, node.outMin, node.outMax);

          if (node.DonM) {
            node._derivative = -1 * node.kd * dInput;
          } else {
            node._derivative = node.kd * error;
          }

          let output = node._proportional + node._integral + node._derivative;
          output = clamp(output, node.outMin, node.outMax);

          node._lastTime = now;
          node._lastOutput = output;
          node._lasInput = input;
          node._lastError = error;
        }

        function autoTune(input) {
          let now = Date.now();
          let ret = false;
          if (input > node._autoTuneMaxPeak[0]) {
            node._autoTuneMaxPeak = [input, now];
          }

          if (input < node._autoTuneMinPeak[0]) {
            node._autoTuneMinPeak = [input, now];
          }
          switch (node._autoTuneState) {
            case IDLE:
              node._lastOutput = node.outMin;
              if (input < node._autoTuneLow) {
                console.log("Going Up");
                node._autoTuneMinPeak = [input, now];
                node._autoTuneMaxPeak = [input, now];
                node._autoTuneState = UP;
                node._lastOutput = node.outMax;
              }
              break;

            case UP:
              if (input > node._autoTuneHigh) {
                if (node._autoTunePeaks.length) {
                  node._autoTunePeaks.push(node._autoTuneMinPeak);
                  node._autoTuneMinPeak = [input, now];
                }
                if (node._autoTunePeaks.length >= 4) {
                  console.log("Calculating Output");
                  node._lastOutput = node.outMin;
                  node._autoTuneState = FINISH;
                } else {
                  console.log("Going Down");
                  node._autoTuneState = DOWN;
                  node._lastOutput = node.outMin;
                }
              }
              break;

            case DOWN:

              if (input < node._autoTuneLow) {
                node._autoTunePeaks.push(node._autoTuneMaxPeak);
                node._autoTuneMaxPeak = [input, now];
                console.log("Going Up");
                node._lastOutput = node.outMax;
                node._autoTuneState = UP;
              }
              break;

            case FINISH:
              let amplitude1 = node._autoTunePeaks[0][0] - node._autoTunePeaks[1][0];
              let amplitude2 = node._autoTunePeaks[2][0] - node._autoTunePeaks[3][0];

              let amplitude = (amplitude1 + amplitude2) / 2;

              node._Ku = 2.0 * (node._autoTuneHigh - node._autoTuneLow) / (amplitude * Math.PI);

              let period1 = node._autoTunePeaks[2][1] - node._autoTunePeaks[0][1];
              let period2 = node._autoTunePeaks[3][1] - node._autoTunePeaks[1][1];

              node._Pu = 0.5 * (period1 + period2) / 1000.0;

              ret = true;

              break;
          }
          return ret;
        }

        reset();

        node.on('input', function(msg, send, done) {
          let newMsg = {};
          if (msg.hasOwnProperty("cmd")) {
            if (msg.cmd === "reset") {
              reset();
            } else if (msg.cmd === "disable") {
              node.enabled = false;
            } else if (msg.cmd === "auto") {
              node.enabled = true;
              setAuto();
            } else if (msg.cmd === "manual") {
              node.enabled = true;
              let output = null;
              if (msg.hasOwnProperty("payload")) {
                output = parseFloat(msg.payload);
              }
              setManual(output);
              newMsg.payload = node._lastOutput;
            } else if (msg.cmd === "autotune") {
              node.enabled = true;
              startAutotune();
            }
          } else if (msg.hasOwnProperty("payload")) {
            if (node.enabled) {
              let tempOutput = parseFloat(msg.payload);
              if (node._autotuneState != OFF) {
                if (!isNaN(tempOutput)) {
                  let ret = autoTune(tempOutput);
                  if (ret) {
                    node._autoTuneState = OFF;
                    console.log("Finished");
                    let divisors = _tuning_rules[node.tuningRule]
                    node.kp = node._Ku / divisors[0]
                    node.ki = node.kp / (node._Pu / divisors[1])
                    node.kd = node.kp * (node._Pu / divisors[2])
                    newMsg.topic = "Autotune Success";
                    newMsg.kp = node.kp;
                    newMsg.ki = node.ki;
                    newMsg.kd = node.kd;
                    console.log(node.kp, node.ki, node.kd);
                  }
                }
              } else if (node.auto) {
                if (!isNaN(tempOutput)) {
                  calculate(tempOutput);
                  newMsg.proportional = node._proportional;
                  newMsg.integral = node._integral;
                  newMsg.derivative = node._derivative;
                }
              }
              newMsg.payload = node._lastOutput;
            }
          }
          send(newMsg);
          if (done) done();
        });
    }
    RED.nodes.registerType("pid-complete",CompletePIDNode);
}
