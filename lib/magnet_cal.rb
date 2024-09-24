require 'ostruct'
require 'thread'
require_relative 'loggable'
require_relative 'green_heron_rt21'
require_relative 'wit_motion_imu'

module Moonraker
  class MagnetCal
    include Loggable

    def initialize(config, data_dir)
      @imu = WitMotionIMU.new(config['imu'], data_dir)
      @az_rotor = GreenHeronRT21.new('AZ', config['azimuth'])
      @el_rotor = GreenHeronRT21.new('EL', config['elevation'])

      @az_rotor.on_heading = ->(heading) { log "AZ @ #{heading}"}
      @el_rotor.on_heading = ->(heading) { log "EL @ #{heading}"}
    end

    def run
      @run_thread = Thread.new do
        log '*** Beginning magnetometer calibration sequence ***'
        
        @imu.start
        @az_rotor.open
        @el_rotor.open

        steps = [
          OpenStruct.new(az_steps: [1, 180, 359], el: 0),
          OpenStruct.new(az_steps: [359, 180, 1], el: 45),
          OpenStruct.new(az_steps: [1, 180, 359], el: 90),
        ]

        steps.each.with_index do |step, step_index|
          log "--- STEP #{step_index + 1} OF #{steps.count} ---"
          
          @az_rotor.stop
          @el_rotor.stop

          @az_rotor.turn(step.az_steps.first)
          @el_rotor.turn(step.el)
          
          log "Waiting for EL rotor to reach #{step.el}°..."
          wait_for_heading(@el_rotor, step.el)

          log "Waiting for AZ rotor to reach #{step.az_steps.first}°..."
          wait_for_heading(@az_rotor, step.az_steps.first)
          
          if step_index == 0
            log 'Starting data collection...'
            @imu.start_calibration
          end

          step.az_steps[1..-1].each do |step_az|
            log "Waiting for AZ rotor to reach #{step_az}°..."
            @az_rotor.turn(step_az)
            wait_for_heading(@az_rotor, step_az)
          end
        end

        @imu.end_calibration
        log '*** Magnetometer calibration sequence complete ***'

        @imu.stop
        @az_rotor.close
        @el_rotor.close
      end

      @run_thread.join
    end

    def abort
      log 'Aborted'

      @az_rotor.stop
      @el_rotor.stop

      @run_thread.kill
      @imu.stop
      @az_rotor.close
      @el_rotor.close
    end
  
    private
  
    def wait_for_heading(rotor, target_heading)
      threshold = 2
  
      while heading_difference(rotor.heading, target_heading).abs > threshold
        sleep 1
      end

      rotor.stop
      sleep 2
    end
  
    def heading_difference(heading1, heading2)
      diff = (heading1 - heading2) % 360
      diff > 180 ? 360 - diff : diff
    end
  end
end