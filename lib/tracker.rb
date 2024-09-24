# ---------- Standard Libs ----------

require 'ostruct'

# ---------- Gems ----------

require 'astronoby'

# ---------- Moonraker Libs ----------

require_relative 'green_heron_rt21'
require_relative 'loggable'
require_relative 'wit_motion_imu'

# ---------- Moonraker Class ----------

module Moonraker
  class Tracker
    include Loggable
    
    def initialize(config, data_dir)
      @config = config

      @observer = Astronoby::Observer.new(
        latitude: Astronoby::Angle.from_degrees(@config['latitude']),
        longitude: Astronoby::Angle.from_degrees(@config['longitude'])
      )

      @az_rotor = GreenHeronRT21.new('AZ', @config['azimuth'])
      @el_rotor = GreenHeronRT21.new('EL', @config['elevation'])
      @imu = WitMotionIMU.new(@config['imu'], data_dir)

      @az_rotor.on_heading = ->(heading) { log "Rotor AZ @ #{heading}°" }
      @el_rotor.on_heading = ->(heading) { log "Rotor EL @ #{heading}°" }

      @imu.on_azimuth = ->(az) {
        if @prev_imu_az == nil || (@prev_imu_az - az).abs >= 0.1
          log "IMU AZ @ #{az.round(1)}°"
          @prev_imu_az = az
        end
      }

      @imu.on_elevation = ->(el) { 
        if @prev_imu_el == nil || (@prev_imu_el - el).abs >= 0.1
          log "IMU EL @ #{el.round(1)}°"
          @prev_imu_el = el
        end
      }
    end

    def start
      @az_rotor.open
      @el_rotor.open
      @imu.start
      
      @moon_position_thread = Thread.new do
        loop do
          moon = Astronoby::Moon.new(time: Time.now)
          coordinates = moon.horizontal_coordinates(observer: @observer)
      
          update_moon_coordinates(OpenStruct.new(az: coordinates.azimuth.degrees, el: coordinates.altitude.degrees))
          # update_moon_coordinates(OpenStruct.new(az: 300, el: 45))
          sleep 1
        end
      end
    end

    def wait
      @moon_position_thread.join
      @imu.stop
      @az_rotor.close
      @el_rotor.close
    end

    def stop
      @moon_position_thread.kill
    end

    private

    def update_moon_coordinates(coords)
      # only update if > 0.1° change
      az_delta = @config['azimuth']['delta']
      el_delta = @config['elevation']['delta']
      return if @prev_coords != nil && (coords.az - @prev_coords.az).abs < az_delta && (coords.el - @prev_coords.el).abs < el_delta
    
      log "Setting AZ: #{coords.az.round(1)}° EL: #{coords.el.round(1)}°"
    
      @az_rotor.turn(coords.az)
      @el_rotor.turn([coords.el, 0].max) # don't go below 0°
    
      @prev_coords = coords
    end
  end
end