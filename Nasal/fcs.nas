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
      return getprop(me.input_path ~ "/" ~ axis);
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
    var status = getprop("/controls/flight/fcs/sas-enabled");
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
  new : func(initial_gains, sensitivities, input_path, output_path) {
    var obj = FCSFilter.new(input_path, output_path);
    obj.parents = [FCSFilter, CAS];
    obj.sensitivities = sensitivities; 
    obj.initial_gains = initial_gains;
    props.globals.getNode("/controls/flight/fcs/gains/cas", 1).setValues(obj.initial_gains);
    setprop("/controls/flight/fcs/cas-enabled", 1);
    return obj;
  },

  calcRollRateAdjustment : func {
    var position = getprop("/orientation/roll-deg");
    return math.sin(position / 180 * math.pi) / 3;
  },
  
  # FIXME: command for CAS is just a temporal one
  calcCommand: func (axis, input) {
    var target_input = 0;
    var gain = me.calcGain(axis);
    var target_rate = input * gain;
    var rate = getprop("/orientation/" ~ axis ~ "-rate-degps");
    var drate = target_rate - rate;
    setprop("/controls/flight/fcs/cas/target_" ~ axis ~ "rate", target_rate);
    setprop("/controls/flight/fcs/cas/delta_" ~ axis, drate);
    if (axis == 'roll') {
      target_input = (drate / gain - me.calcRollRateAdjustment());
      setprop("/controlss/flight/fcs/gains/cas/rollAdjust", me.calcRollRateAdjustment());
    } else {
      target_input = drate / gain;
    }
    return target_input;
  },

  # FixMe: gain should be calculated using both speed and dynamic pressure
  calcGain : func(axis) {
    var mach = getprop("/velocities/mach");
    var initial_gain = getprop("/controls/flight/fcs/gains/cas/" ~ axis);
    var gain = initial_gain;
    if (axis == 'pitch') {
      gain += 0.1 * mach * mach;
    } elsif (axis== 'yaw') {
      gain *= (mach * mach);
    }
    if (gain * initial_gain < 0.0 ) {
      gain = 0;
    }
    return gain;
  }, 

  apply : func(axis) {
    var input = me.read(axis);
    var status = getprop("/controls/flight/fcs/cas-enabled");
    if (status == 0) {
      me.write(axis, input);
      return;
    }
    var cas_command = me.calcCommand(axis, input);
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
                   #   0   10     20   30   40   50    60   70   80   90  100  110  120  130  140  150  160, 170, 180, .....
   me.gainTable = [-0.9, -0.8, -0.7, -0.5, 0.0, 0.60, 0.75, 0.9, 1.0, 1.0, 0.8, 0.7, 0.6, 0.5, 0.4, 0.1, -0.1, -0.3, -0.5, -1.0];
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

#    if (gain > 0) {
#      stabilator_norm = - gain * math.sqrt(math.abs(mach) * 3.3) + 0.7;
#    } elsif (gain < 0) {
#      stabilator_norm = - gain * math.sqrt(math.abs(mach) * 3.3) - 0.7;
#    } else {
#      stabilator_norm = 0;
#    }
#    
#    if (mach < 0) {
#      stabilator_norm = - stabilator_norm; 
#    }
#    if (stabilator_norm > 1) {
#      stabilator_norm = 1.0;
#    } elsif (stabilator_norm < -1) {
#      stabilator_norm = -1.0;
#    }
#
    setprop("/controls/flight/fcs/stabilator", stabilator_norm);
  }
};

var sas = nil;
var cas = nil;
var stabilator = nil;

var sensitivities = {'roll' : 0.5, 'pitch' : 0.3, 'yaw' : 0.2 };
var sas_initial_gains = {'roll' : 0.02, 'pitch' : -0.10, 'yaw' : 0.04 };
var cas_initial_gains = {'roll' : 30, 'pitch' : -20, 'yaw' : 35 };
#var cas_initial_gains = {'roll' : 10, 'pitch' : -10, 'yaw' : 10 };

var update = func {
  if (cas != nil) {
    cas.apply('roll');
    cas.apply('pitch');
    cas.apply('yaw');
  }
  sas.apply('roll');
  sas.apply('pitch');
  sas.apply('yaw');
  stabilator.update();
  settimer(update, 0);
}

var initialize = func {
#  cas = CAS.new(cas_initial_gains, sensitivities, nil, "/controls/flight/fcs/cas");
#  sas = SAS.new(sas_initial_gains, sensitivities, 0.4, "/controls/flight/fcs/cas", "/controls/flight/fcs");
  sas = SAS.new(sas_initial_gains, sensitivities, 0.35, nil, "/controls/flight/fcs");
  stabilator = Stabilator.new();
  settimer(update, 0);
}

_setlistener("/sim/signals/fdm-initialized", initialize);

