#
# Flight Control System by Tatsuhiro Nishioka 
# $Id$
#

var FCSFilter = {
  new : func(input_path, output_path) {
    var obj = { parents : [FCSFilter], 
                input_path : input_path,
                output_path : output_path };
    obj.axis_conv = {'roll' : 'aileron', 'pitch' : 'elevator', 'yaw' : 'rudder' };
    return obj;
  },

  # read input command for a given axis
  read : func(axis) {
    if (me.input_path == nil or me.input_path == "") {
      return getprop("/controls/flight/" ~ me.axis_conv[axis]);
    } else { 
      var value = getprop(me.input_path ~ "/" ~ axis);
      value = int(value * 100) / 100.0;
    }
  },

  # write output command for a given axis
  # this will be the output of an next command filter (like SAS)
  write : func(axis, value) {
    if (me.output_path == nil or me.output_path == '') {
      setprop("/controls/flight/fcs/" ~ axis, value);
    } else {
      setprop(me.output_path ~ "/" ~ axis, value);
    }
  },

  toggleFilterStatus : func(name) {
    var messages = ["disengaged", "engaged"];
    var path = "/controls/flight/fcs/" ~ name ~ "-enabled";
    var status = getprop(path);
    setprop(path, 1 - status);
    screen.log.write(name ~ " " ~ messages[1 - status]);
  },

  getStatus : func(name) {
    var path = "/controls/flight/fcs/" ~ name ~ "-enabled";
    return getprop(path);
  },

  limit : func(value, range) {
    if (value > range) {
      return range;
    } elsif (value < -range) {
      return - range;
    }
    return value;
  }
};


# 
# SAS : Stability Augmentation System - a rate damper
# 
var SAS = {
  # 
  # new
  #   initial_gains: hash of initial gains for rate damping
  #   sensitivities: hash of minimum rates (deg/sec) that enables rate damper
  #   authority_limit: shows how much SAS can take over control
  #                    0 means no stability control, 1.0 means SAS fully takes over pilot control
  #   input_path: is a base path to input axis; nil for using raw input from KB/JS
  #   output_path: is a base path to output axis; nis for using /controls/flight/fcs
  # 
  #   with input_path / output_path, you can connect SAS, CAS, and more control filters
  #
  new : func(initial_gains, sensitivities, authority_limit, input_path, output_path) {
    var obj = FCSFilter.new(input_path, output_path);
    obj.parents = [FCSFilter, SAS];
    obj.authority_limit = authority_limit;
    obj.sensitivities = sensitivities; 
    obj.initial_gains = initial_gains;
    props.globals.getNode("/controls/flight/fcs/gains/sas", 1).setValues(obj.initial_gains);
    setprop("/controls/flight/fcs/sas-enabled", 1);
    return obj;
  },

  toggleEnable : func() {
    me.toggleFilterStatus("sas");
  },

  # 
  # calcGain - get gain for each axis based on air speed and dynamic pressure
  #   axis: one of 'roll', 'pitch', or 'yaw'
  # 
  calcGain : func(axis) {
    var mach = getprop("/velocities/mach");
#    var initial_gain = me.initial_gains[axis];
    var initial_gain = getprop("/controls/flight/fcs/gains/sas/" ~ axis);
    var gain = initial_gain - 0.1 * mach * mach;
    if (math.abs(gain) < math.abs(initial_gain) * 0.01 or gain * initial_gain < 0) {
      gain = initial_gain * 0.01;
    }
    return gain;
  }, 

  calcAuthorityLimit : func() {
    var mach = getprop("/velocities/mach");
    var min_mach = 0.038;
    var limit = me.authority_limit;
    if (math.abs(mach < min_mach)) {
      limit += (min_mach - math.abs(mach))  / min_mach * (1 - me.authority_limit) * 0.95;
    }
    setprop("/controls/flight/fcs/gains/sas/authority-limit", limit);
    return limit;
  },

  # 
  # apply - apply SAS damper to a given input axis
  #   axis: one of 'roll', 'pitch', or 'yaw'
  # 
  apply : func(axis) {
    var status = me.getStatus("sas");
    var input = me.read(axis);
    if (status == 0) {
      me.write(axis, input);
      return;
    }
    var mach = getprop("/velocities/mach");
    var value = 0;
    var rate = getprop("/orientation/" ~ axis ~ "-rate-degps");
    var gain = me.calcGain(axis);
    var limit = me.calcAuthorityLimit();
    if (math.abs(rate) >= me.sensitivities[axis]) {
      value = - gain * rate;
      if (value > limit) {
        value = limit;
      } elsif (value < - limit) {
        value = - limit;
      } 
    }
    me.write(axis, value + input);
  }
};

# 
# CAS : Control Augmentation System - makes your aircraft more meneuverable
# 
var CAS = {
  new : func(input_gains, output_gains, sensitivities, input_path, output_path) {
    var obj = FCSFilter.new(input_path, output_path);
    obj.parents = [FCSFilter, CAS];
    obj.sensitivities = sensitivities; 
    obj.input_gains = input_gains;
    obj.output_gains = output_gains;
    obj.last_body_fps = {'roll' : 0, 'pitch' : 0};
    obj.last_pos = {'roll' : 0.0, 'pitch' : 0.0, 'yaw' : 0.0};
    props.globals.getNode("/controls/flight/fcs/gains/cas/input", 1).setValues(obj.input_gains);
    props.globals.getNode("/controls/flight/fcs/gains/cas/output", 1).setValues(obj.output_gains);
    setprop("/controls/flight/fcs/cas-enabled", 1);
    setprop("/controls/flight/fcs/auto-hover-enabled", 0);
    setprop("/controls/flight/fcs/gains/cas/fps-reaction-gain-roll", 0.4);
    setprop("/controls/flight/fcs/gains/cas/fps-reaction-gain-pitch", -1.9);
    setprop("/controls/flight/fcs/gains/cas/fps-roll-coeff", 0.5);
    setprop("/controls/flight/fcs/gains/cas/fps-roll-brake-freq", 3);
    setprop("/controls/flight/fcs/gains/cas/fps-pitch-coeff", 0.5);
    setprop("/controls/flight/fcs/gains/cas/fps-pitch-brake-freq", 0.6);
    return obj;
  },

  calcRollRateAdjustment : func {
    var position = getprop("/orientation/roll-deg");
    return math.abs(math.sin(position / 180 * math.pi)) / 6;
  },
  
  # FIXME: command for CAS is just a temporal one
  calcCommand: func (axis, input) {
    var output = 0;
    var input_gain = me.calcGain(axis);
    var output_gain = getprop("/controls/flight/fcs/gains/cas/output/" ~ axis);
    var target_rate = input * input_gain;
    var rate = getprop("/orientation/" ~ axis ~ "-rate-degps");
    var drate = target_rate - rate;
    setprop("/controls/flight/fcs/cas/target_" ~ axis ~ "rate", target_rate);
    setprop("/controls/flight/fcs/cas/delta_" ~ axis, drate);
    if (axis == 'roll') {
       if (math.abs(input > 0.7)) {
         output = input; # (drate * output_gain - me.calcRollRateAdjustment());
         setprop("/controls/flight/fcs/gains/cas/rollAdjust", me.calcRollRateAdjustment());
       } else {
         var target_deg = (input * 90) / 0.7;
         var roll_deg = getprop("/orientation/roll-deg");
         var ddeg = target_deg - roll_deg;
         output = ddeg * output_gain;
      }
      output = output + me.calcCounterVBodyFPS(input);
    } else {
      output = drate * output_gain;
    }
    return output;
  },

  toggleEnable : func() {
    me.toggleFilterStatus("cas");
  },

  toggleAutoHover : func() {
    me.toggleFilterStatus("auto-hover");
  },

  # 
  # auto hover
  # 
  autoHover : func(axis, input) {
    var position = getprop("/orientation/" ~ axis ~ "-deg");
    var output_gain = getprop("/controls/flight/fcs/gains/cas/output/" ~ axis);
    var body_fps = 0;
    var last_body_fps = me.last_body_fps[axis];
    var reaction_gain = 0;
    var heading = getprop("/orientation/heading-deg");
    var wind_speed_fps = getprop("/environment/wind-speed-kt") * 1.6878099;
    var wind_direction = getprop("/environment/wind-from-heading-deg");
    var wind_direction -= heading;
    var rate = getprop("/orientation/" ~ axis ~ "-rate-degps");
#    var ddeg = me.last_pos[axis] - position;
    var gear_pos = getprop("/gear/gear[0]/compression-norm") + getprop("/gear/gear[1]/compression-norm");
    var counter_fps = 0;
    if (axis == 'roll') {
      var target_pos = -0.8;
      var brake_deg = 0;
      body_fps = getprop("/velocities/vBody-fps");
      wind_fps = math.sin(wind_direction / 180 * math.pi) * wind_speed_fps; 
      var brake_freq = getprop("/controls/flight/fcs/gains/cas/fps-roll-brake-freq");
      body_fps -= wind_fps;
      var dfps = body_fps - me.last_body_fps[axis];
      var fps_roll_coeff = getprop("/controls/flight/fcs/gains/cas/fps-roll-coeff");
      target_pos -= int(body_fps * 100) / 100 * fps_roll_coeff;
#      target_pos += wind_fps * fps_roll_coeff;
#      target_pos -= fps_roll_coeff * me.limit(body_fps * 6, 15);
      if (gear_pos > 0.0 and position > 0) {
        target_pos -= position * gear_pos / 5;
      }
      reaction_gain = getprop("/controls/flight/fcs/gains/cas/fps-reaction-gain-roll");
      if (math.abs(position + rate / brake_freq * 2) > math.abs(target_pos)) {
        if (math.abs(dfps) > 1) {
          dfps = 1;
        }
        brake_deg = (target_pos - (position + rate / brake_freq)) * math.abs(dfps * 10) / brake_freq;
      }
      var roll_reaction_gain = getprop("/controls/flight/fcs/gains/cas/reaction-gain-roll");
      counter_fps = (target_pos + brake_deg) * reaction_gain;
      # tmporary
      output_gain = 0;
      setprop("/controls/flight/fcs/cas/ah-vbody-fps", body_fps);
      setprop("/controls/flight/fcs/cas/ah-vbody-wind-fps", wind_fps);
      setprop("/controls/flight/fcs/cas/ah-roll-target-deg", target_pos);
      setprop("/controls/flight/fcs/cas/ah-roll-rate", rate);
      setprop("/controls/flight/fcs/cas/ah-delta-vbodyfps", dfps);
      setprop("/controls/flight/fcs/cas/ah-roll-brake-deg", brake_deg);
      
    } elsif (axis == 'pitch') {
      var target_pos = 0;
      var brake_deg = 0;
      body_fps = getprop("/velocities/uBody-fps");
      var wind_fps = math.cos(wind_direction / 180 * math.pi) * wind_speed_fps;
      body_fps -= wind_fps;
      var brake_freq = getprop("/controls/flight/fcs/gains/cas/fps-pitch-brake-freq");
      var dfps = body_fps - me.last_body_fps[axis];
      var fps_coeff = getprop("/controls/flight/fcs/gains/cas/fps-pitch-coeff");
      target_pos += int(body_fps * 10) / 10 * fps_coeff;

      if (math.abs(position + rate / brake_freq * 5) > math.abs(target_pos)) {
        if (math.abs(dfps) > 1) {
          dfps = 1;
        }
        brake_deg = (target_pos - (position + rate / brake_freq)) * math.abs(dfps * 10) / brake_freq;
      }

      reaction_gain = getprop("/controls/flight/fcs/gains/cas/fps-reaction-gain-pitch");
      counter_fps = (body_fps + brake_deg) * reaction_gain;
      setprop("/controls/flight/fcs/cas/ah-ubody-fps", body_fps);
      setprop("/controls/flight/fcs/cas/ah-ubody-wind-fps", wind_fps);
      setprop("/controls/flight/fcs/cas/ah-pitch-target-deg", target_pos);
      setprop("/controls/flight/fcs/cas/ah-pitch-rate", rate);
      setprop("/controls/flight/fcs/cas/ah-delta-ubodyfps", dfps);
      setprop("/controls/flight/fcs/cas/ah-pitch-brake-deg", brake_deg);
    } else {
      return me.calcCommand(axis, input);
    }
    counter_fps = me.limit(counter_fps, 1.0);
    setprop("/controls/flight/fcs/cas/counter-fps-" ~ axis, counter_fps);
    me.last_pos[axis] = position;
    me.last_body_fps[axis] = body_fps;
    return me.limit(counter_fps + input * 0.2, 1.0);
  },

  calcCounterVBodyFPS : func(input) {
    var limit = 0.2;
    if (math.abs(input) > 0.1) {
      return 0;
    }
    var output_gain = getprop("/controls/flight/fcs/gains/cas/output/roll");
    var heading = getprop("/orientation/heading-deg");
    var wind_speed_fps = getprop("/environment/wind-speed-kt") * 1.6878099;
    var wind_direction = getprop("/environment/wind-from-heading-deg");
    var wind_direction -= heading;
    var vbody_fps = getprop("/velocities/vBody-fps");
    vbody_fps -= math.sin(wind_direction / 180 * math.pi) * wind_speed_fps;      
    var counterReaction = - vbody_fps * 1.5 * output_gain;
    setprop("/controls/flight/fcs/counter-reaction-vbody-fps", counterReaction);
    if (counterReaction > 0.2) {
      counterReaction = 0.2;
    } elsif (counterReaction < -0.2) {
      counterReaction = -0.2;
    }
    return counterReaction ;
  },

  # FixMe: gain should be calculated using both speed and dynamic pressure
  calcGain : func(axis) {
    var mach = getprop("/velocities/mach");
    var input_gain = getprop("/controls/flight/fcs/gains/cas/input/" ~ axis);
    var gain = input_gain;
    if (axis == 'pitch') {
      gain += 0.1 * mach * mach;
    } elsif (axis== 'yaw') {
      gain *= ((1 - mach) * (1 - mach));
    }
    if (gain * input_gain < 0.0 ) {
      gain = 0;
    }
    return gain;
  }, 

  apply : func(axis) {
    var input = me.read(axis);
    var hover = me.getStatus("auto-hover");
    var status = me.getStatus("cas");
    var cas_command = 0;
    if (status == 0) {
      me.write(axis, input);
      return;
    }
    if (hover == 1) {
      cas_command = me.autoHover(axis, input);
    } else {
      cas_command = me.calcCommand(axis, input);
    }
    me.write(axis, cas_command);
  }
};

#
# Tail hstab, "stabilator," for stabilize the nose 
#
var Stabilator = {
  new : func() {
    var obj = { parents : [Stabilator] };
    setprop("/controls/flight/fcs/gains/stabilator", -1.8);
    setprop("/controls/flight/fcs/auto-stabilator", 1);
                   #   0    10   20    30   40   50   60   70   80   90  100  110  120  130  140  150  160, 170, 180, .....
   me.gainTable =  [-0.9, -0.8, 0.1, -0.5, 0.0, 0.7, 0.8, 0.9, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.9, 0.8, 0.6, 0.4, 0.2, -1.0];
#   me.gainTable = [-0.9, -0.8, -0.7, -0.5, 0.0, 0.60, 0.75, 0.9, 1.0, 1.0, 0.8, 0.7, 0.6, 0.5, 0.4, 0.1, -0.1, -0.3, -0.5, -1.0];
#   me.gainTable = [-0.9, -0.8, -0.7, -0.5, 0.0, 0.60, 0.75, 0.9, 1.0, 1.0, 1.0, 0.9, 0.8, 0.7, 0.5, 0.3, 0.1, -0.2, -0.4, -1.0];
    return obj;
  },

  toggleManual : func {
    var status = getprop("/controls/flight/fcs/auto-stabilator");
    getprop("/controls/flight/fcs/auto-stabilator", 1 - status);
  },
  
  apply : func(delta) {
    setprop("/controls/flight/fcs/auto-stabilator", 0);
    var value = getprop("/controls/flight/fcs/stabilator");
    getprop("/controls/flight/fcs/stabilator", value + delta);
  },

  calcPosition : func() {
    var speed = getprop("/velocities/mach") / 0.001497219; # in knot
    var index = int(math.abs(speed) / 10);
    if (index >= size(me.gainTable) - 1) {
      index = size(me.gainTable) - 2;
    }
    var mod = math.mod(int(math.abs(speed)), 10);
    var position = me.gainTable[index] * ((10 - mod) / 10) + me.gainTable[index-1] * (mod) / 10;
    if (speed < -20) {
      position = - position;
    }
    return position;
  },

  update : func() {
    var status = getprop("/controls/flight/fcs/auto-stabilator");
    if (status == 0) {
      return;
    }
    var gain = getprop("/controls/flight/fcs/gains/stabilator");
    var mach = getprop("/velocities/mach");
    var throttle = getprop("/controls/flight/throttle");
    var stabilator_norm = 0;

    stabilator_norm = me.calcPosition();   
    setprop("/controls/flight/fcs/stabilator", stabilator_norm);
  }
};

var sas = nil;
var cas = nil;
var stabilator = nil;
var count = 0;

# var sensitivities = {'roll' : 2.0, 'pitch' : 1.0, 'yaw' : 1.0 };
var sensitivities = {'roll' : 1.0, 'pitch' : 1.0, 'yaw' : 3.0 };
var sas_initial_gains = {'roll' : 0.02, 'pitch' : -0.10, 'yaw' : 0.04 };
var cas_input_gains = {'roll' : 30, 'pitch' : -30, 'yaw' : 35 };
var cas_output_gains = {'roll' : 0.02, 'pitch' : -0.5, 'yaw' : 6.0 };

var update = func {
  count += 1;
  if (math.mod(count, 2) == 0) {
    return;
  }
  if (cas != nil) {
    cas.apply('roll');
    cas.apply('pitch');
    cas.apply('yaw');
  }
  sas.apply('roll');
  sas.apply('pitch');
  sas.apply('yaw');
  stabilator.update();
#  settimer(update, 0);
}

var initialize = func {
  cas = CAS.new(cas_input_gains, cas_output_gains, sensitivities, nil, "/controls/flight/fcs/cas");
  sas = SAS.new(sas_initial_gains, sensitivities, 0.2, "/controls/flight/fcs/cas", "/controls/flight/fcs");
#  sas = SAS.new(sas_initial_gains, sensitivities, 0.3, nil, "/controls/flight/fcs");
  stabilator = Stabilator.new();
#  settimer(update, 0);
  setlistener("/rotors/main/cone-deg", update);
}

_setlistener("/sim/signals/fdm-initialized", initialize);

