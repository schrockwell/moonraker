# ---------- Standard library ----------
require 'thread'

# ---------- Gems ----------

require 'uart'

module Moonraker
  class WitMotionClient
    def initialize(port, baud, data_dir)
      @port = port
      @baud = baud
      @data_dir = data_dir

      @cal_mutex = Mutex.new

      load_calibration
    end
    
    def start(&callback)
      @heading_callback = callback

      data_bits = 8
      stop_bits = 1
      parity = SerialPort::NONE

      @serial = SerialPort.new(@port, @baud, data_bits, stop_bits, parity)
      read_thread
    end

    def start_calibration
      @cal_mutex.synchronize do
        return if @calibrating

        @min_x = nil
        @max_x = nil
        @min_y = nil
        @max_y = nil
        @min_z = nil
        @max_z = nil

        @calibrating = true
        @cal_data_file = File.open('/data/wit-cal-data.csv', 'w')

        puts 'Starting calibration...'
      end
    end

    def end_calibration
      @cal_mutex.synchronize do
        return unless @calibrating

        @calibrating = false
        @cal_data_file.close

        puts 'Computing calibration...'
        
        save_calibration({
          'x_offset' => -(@min_x + @max_x) / 2,
          'y_offset' => -(@min_y + @max_y) / 2,
          'z_offset' => -(@min_z + @max_z) / 2
        })
      end
    end

    def calibrating?
      @calibrating
    end

    def read_thread
      Thread.new do
        buffer = ''
        begin
          loop do
            while (byte = @serial.getbyte)
              if buffer.empty? && byte == 0x55
                # Start of a new message
                buffer << byte.chr
              elsif !buffer.empty? && buffer.bytesize < 11
                # Continue reading message bytes
                buffer << byte.chr
              elsif buffer.bytesize >= 11
                # We have a complete message; parse it
                heading = parse_message(buffer)
                @heading_callback.call(heading) if heading

                buffer = byte == 0x55 ? byte.chr : "" # Start a new message if we ended with 0x55
              end
            end
          end
        rescue => e
          puts "Error reading from serial port: #{e.message}"
        end
      end          
    end

    private

    def calibration_path
      File.join(@data_dir, 'wit-cal.toml')
    end

    def load_calibration
      if File.exist?(calibration_path)
        @calibration = TOML.load_file(calibration_path)
        puts 'Loaded magnetometer calibration data.'
      else
        puts '*** WARNING! *** No calibration data for magnetometer. Using default values.'

        @calibration = {
          'x_offset' => 0,
          'y_offset' => 0,
          'z_offset' => 0
        }
      end
    end

    def save_calibration(new_calibration)
      @calibration = new_calibration

      File.open(calibration_path, 'w') do |f|
        f.write(TOML::Generator.new(@calibration).body)
      end

      put 'Saved calibration data.'
    end

    def parse_message(buffer)
      # Ensure buffer is of the correct size (1 for 0x54 + 8 for data + 1 for checksum)
      return unless buffer.size == 11
      return unless buffer.bytes[1] == 0x54
    
      # Calculate checksum (excluding the last byte of the buffer)
      calculated_checksum = buffer.bytes[0..-2].reduce(:+) & 0xFF # Ensuring it's a byte value
    
      # Extract the checksum from the message
      message_checksum = buffer.bytes.last
    
      if calculated_checksum != message_checksum
        return
      end
    
      # Parse X, Y, Z, and Temperature from the buffer (little-endian)
      x, y, z = buffer[2..7].unpack('s<s<s<') # 16-bit signed integers, little-endian
    
      @cal_mutex.synchronize do
        if @calibrating
          @max_x = [x, @max_x].compact.max
          @min_x = [x, @min_x].compact.min
          @max_y = [y, @max_y].compact.max
          @min_y = [y, @min_y].compact.min
          @max_z = [z, @max_z].compact.max
          @min_z = [z, @min_z].compact.min

          @cal_data_file.puts("#{x},#{y},#{z}")

          return
        end
      end
    
      # Apply the known offsets
      x = x + @calibration['x_offset']
      y = y + @calibration['y_offset']
      z = z + @calibration['z_offset']
      
      # Sensor is placed with the mounting plate facing the ground, and the +Y axis pointing down the
      # boom of the antenna.
      heading_degrees = Math.atan2(-x, y) * (180.0 / Math::PI)
      (heading_degrees + 360) % 360
    end
  end
end