# ---------- Standard Libs ----------

require 'ostruct'

# ---------- Gems ----------

require 'astronoby'
require 'uart'

# ---------- Moonraker Class ----------

module Moonraker
  class Tracker
    def initialize(config)
      @config = config

      @observer = Astronoby::Observer.new(
        latitude: Astronoby::Angle.from_degrees(@config['latitude']),
        longitude: Astronoby::Angle.from_degrees(@config['longitude'])
      )
    end

    def start
      raise "Azimuth rotor not found: #{@config['azimuth']['port']}" unless File.exist? @config['azimuth']['port']
      @az_uart = UART.open @config['azimuth']['port'], @config['azimuth']['baud']
      log "Opened azimuth port #{@config['azimuth']['port']} at #{@config['azimuth']['baud']} baud"
      
      raise "Elevation rotor not found: #{@config['elevation']['port']}" unless File.exist? @config['elevation']['port']
      @el_uart = UART.open @config['elevation']['port'], @config['elevation']['baud']
      log "Opened elevation port #{@config['elevation']['port']} at #{@config['elevation']['baud']} baud"
      
      @moon_position_thread = Thread.new do
        loop do
          moon = Astronoby::Moon.new(time: Time.now)
          coordinates = moon.horizontal_coordinates(observer: @observer)
      
          update_moon_coordinates(OpenStruct.new(az: coordinates.azimuth.degrees, el: coordinates.altitude.degrees))
          sleep 1
        end
      end
    end

    def wait
      @moon_position_thread.join
      @az_uart.close if @az_uart
      @el_uart.close if @el_uart
    end

    def stop
      @moon_position_thread.kill
    end

    private

    def log(msg)
      puts "[#{Time.now.utc}] #{msg}"
    end

    def update_moon_coordinates(coords)
      # only update if > 0.1° change
      az_delta = @config['azimuth']['delta']
      el_delta = @config['elevation']['delta']
      return if @prev_coords != nil && (coords.az - @prev_coords.az).abs < az_delta && (coords.el - @prev_coords.el).abs < el_delta
    
      log "AZ: #{coords.az.round(1)}° EL: #{coords.el.round(1)}°"
    
      az_index = @config['azimuth']['index'] || 1
      el_index = @config['elevation']['index'] || 1

      az_degrees = coords.az.round(1)
      el_degrees = [coords.el.round(1), 0].max # don't go below 0°
    
      az_command = "AP#{az_index}#{az_degrees}\r;"
      el_command = "AP#{el_index}#{el_degrees}\r;"
    
      log "Sending AZ command: #{az_command.inspect}"
      log "Sending EL command: #{el_command.inspect}"

      @az_uart.write(az_command) if @az_uart
      @el_uart.write(el_command) if @el_uart
    
      @prev_coords = coords
    end
  end
end