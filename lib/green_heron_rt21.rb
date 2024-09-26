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

    attr_reader :heading
    attr_accessor :on_heading

    def initialize(name, config, opts = {})
      @name = name
      @port = config['port']
      @baud = config['baud']
      @index = config['index']
      @heading = nil
      @on_heading = nil
      @poll_interval = opts[:poll_interval] || 1.0

      raise "#{@name} rotor not found: #{@port}" unless File.exist?(@port)
    end

    def open
      @uart = UART.open(@port, @baud)
      log "Opened #{@name} rotor"

      read_heading
      
      @poll_thread = poll_thread
    end

    def turn(heading)
      degrees = '%05.1f' % heading
      command = "AP#{@index}#{degrees}\r;"
      @uart.write(command)
    end

    def set_over_travel(degrees)
      @uart.write("WU1#{degrees};")
      @uart.read_next_string_command
    end

    def set_cw_limit(degrees)
      @uart.write("WI1#{degrees};")
      @uart.read_next_string_command
    end

    def set_ccw_limit(degrees)
      @uart.write("WH1#{degrees};")
      @uart.read_next_string_command
    end

    def stop
      @uart.write(";")
    end

    def close
      @poll_thread.kill
      @uart.close
    end

    private

    def poll_thread
      Thread.new do
        loop do
          read_heading
          sleep @poll_interval
        end
      end
    end
    
    def read_heading
      @uart.write("BI#{@index};")
      response = @uart.read_next_string_command
      return unless response.match?(/\d{1,3}\.\d;$/)

      prev_heading = @heading
      @heading = response.to_f
      @on_heading&.call(@heading) if @heading != prev_heading
    end
  end
end