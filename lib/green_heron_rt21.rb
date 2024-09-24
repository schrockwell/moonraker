# Standard libs
require 'thread'

# Gems
require 'uart'

# Moonraker libs
require_relative 'io'
require_relative 'loggable'

module Moonraker
  class GreenHeronRT21
    include Loggable

    def initialize(name, config)
      @name = name
      @port = config['port']
      @baud = config['baud']
      @index = config['index']
      @current_angle = nil

      raise "#{@name} rotor not found: #{@port}" unless File.exist?(@port)
    end

    def open
      @uart = UART.open(@port, @baud)
      log "Opened #{@name} rotor"

      @poll_thread = poll_thread
    end

    def turn(angle)
      degrees = '%05.1f' % angle
      command = "AP#{@index}#{degrees}\r;"
      @uart.write(command)
    end

    def close
      @poll_thread.kill
      @uart.close
    end

    private

    def poll_thread
      Thread.new do
        loop do
          @uart.write("BI#{@index};")
          response = @uart.read_next_string_command(';')
          @current_angle = response.to_f
          sleep 1
        end
      end
    end
  end
end