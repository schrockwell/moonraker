# ---------- Standard library ----------

require 'thread'

# ---------- Gems ----------

require 'uart'

require_relative 'loggable'

module Moonraker
  class WitMotionIMU
    include Loggable

    attr_accessor :on_azimuth, :on_elevation

    def initialize(config, data_dir)
      @port = config['port'] || raise('port is required')
      @baud = config['baud'] || raise('baud is required')
      @declination = config['magnetic_declination'] || raise('magnetic_declination is required')
      @data_dir = data_dir

      @roll_degrees = 0.0
      @pitch_degrees = 0.0
      @yaw_degrees = 0.0

      @cal_mutex = Mutex.new
      @on_azimuth = nil
      @on_elevation = nil

      load_calibration
    end
    
    def start
      @uart = UART.open(@port, @baud)
      log 'Opened IMU'
      @read_thread = read_thread
    end

    def stop
      @read_thread.kill
      @uart.close
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
        @cal_data_file = File.open(File.join(@data_dir, "wit-cal-data-#{Time.now.to_i}.csv"), 'w')
        @cal_data_file.puts('x,y,z,roll,pitch,yaw')

        log 'Starting calibration...'
      end
    end

    def end_calibration
      @cal_mutex.synchronize do
        return unless @calibrating

        @calibrating = false
        @cal_data_file.close

        log 'Computing calibration...'
        
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
            while (byte = @uart.getbyte)
              if buffer.empty? && byte == 0x55
                # Start of a new message
                buffer << byte.chr
              elsif !buffer.empty? && buffer.bytesize < 11
                # Continue reading message bytes
                buffer << byte.chr
              elsif buffer.bytesize >= 11
                # We have a complete message; parse it
                azimuth = parse_magnet_message(buffer)
                @on_azimuth&.call(azimuth) if azimuth

                elevation = parse_angle_message(buffer)
                @on_elevation&.call(elevation) if elevation

                buffer = byte == 0x55 ? byte.chr : "" # Start a new message if we ended with 0x55
              end
            end
          end
        rescue => e
          log "Error reading from uart port: #{e.message}"
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
        log 'Loaded magnetometer calibration data.'
      else
        log '*** WARNING! *** No calibration data for magnetometer. Using default values.'

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

      log 'Saved calibration data.'
    end

    def parse_magnet_message(buffer)
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

          @cal_data_file.puts("#{x},#{y},#{z},#{@roll_degrees},#{@pitch_degrees},#{@yaw_degrees}")

          return
        end
      end
    
      # Apply the known offsets
      x = x + @calibration['x_offset']
      y = y + @calibration['y_offset']
      z = z + @calibration['z_offset']
      
      # Sensor is placed with the mounting plate facing the ground, and the +Y axis pointing down the
      # boom of the antenna.

      # puts "Pitch: #{@pitch_degrees}"
      # puts "Roll: #{@roll_degrees}"

      azimuth_degrees = calculate_azimuth(x, y, z, @pitch_degrees / 180.0 * Math::PI, @roll_degrees / 180.0 * Math::PI)
      magnetic_heading = (azimuth_degrees + 360) % 360

      true_heading = magnetic_heading + @declination
      true_heading
    end

    def calculate_azimuth(x, y, z, pitch, roll)
      # Step 1: Compensate for roll (around x-axis)
      y_prime = y * Math.cos(roll) - z * Math.sin(roll)
      z_prime = y * Math.sin(roll) + z * Math.cos(roll)
    
      # Step 2: Compensate for pitch (around y-axis)
      x_prime = x * Math.cos(pitch) + z_prime * Math.sin(pitch)
      z_double_prime = -x * Math.sin(pitch) + z_prime * Math.cos(pitch)
    
      # Step 3: Calculate azimuth using the corrected x' and y'
      # p [x / x_prime, y / y_prime]
      azimuth_radians = Math.atan2(-x_prime, y_prime)
    
      # Convert radians to degrees
      azimuth_degrees = azimuth_radians * (180.0 / Math::PI)
    
      # Return the azimuth in degrees
      azimuth_degrees
    end

    def parse_angle_message(buffer)
      # Ensure buffer is of the correct size (1 for 0x53 + 8 for data + 1 for checksum)
      return unless buffer.size == 11
      return unless buffer.bytes[1] == 0x53
    
      # Calculate checksum (excluding the last byte of the buffer)
      calculated_checksum = buffer.bytes[0..-2].reduce(:+) & 0xFF # Ensuring it's a byte value
    
      # Extract the checksum from the message
      message_checksum = buffer.bytes.last
    
      if calculated_checksum != message_checksum
        return
      end
    
      # Roll angle X=((RollH<<8)|RollL)/32768*180(°)
      # Pitch angleY=((PitchH<<8)|PitchL)/32768*180(°)
      # Yaw angleZ=((YawH<<8)|YawL)/32768*180(°)

      roll, pitch, yaw = buffer[2..7].unpack('s<s<s<') # 16-bit signed integers, little-endian

      @roll_degrees = roll / 32768.0 * 180
      @pitch_degrees = pitch / 32768.0 * 180
      @yaw_degrees = yaw / 32768.0 * 180

      @roll_degrees
    end
  end
end
