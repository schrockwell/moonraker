# ---------- Standard library ----------

require 'thread'

# ---------- Gems ----------

require 'matrix'
require 'tomlib'
require 'uart'

require_relative 'loggable'

module Moonraker
  class WitMotionIMU
    class Calibration
      def initialize(offset = nil, scaling = nil, rotation = nil)
        @offset = offset || [0, 0, 0]
        @scaling = scaling || [1, 1, 1]
        @rotation = rotation || Matrix.identity(3)
      end

      def save(path)
        calibration = {
          'offset' => @offset.to_a,
          'scaling' => @scaling.to_a,
          'rotation' => @rotation.to_a
        }

        File.open(path, 'w') do |f|
          f.write(Tomlib.dump(calibration))
        end
      end

      def self.load_file(path)
        calibration = Tomlib.load(File.read(path))
        
        # IMPORTANT! Note the splat (*) operator here
        self.new(
          calibration['offset'],
          calibration['scaling'],
          Matrix[*calibration['rotation']],
        )
      end

      def self.parse_measurements_file(path)
        File.read(path).split("\n").map { |l| l.split(",").map(&:to_f) }
      end

      def self.from_measurements_file(path)
        from_measurements(parse_measurements_file(path))
      end

      # Fit an ellipsoid to the 3D points
      def self.fit_ellipsoid(points)
        # Each point (x, y, z) generates one row of a matrix equation
        design_matrix = points.map do |x, y, z|
          [x**2, y**2, z**2, 2 * x * y, 2 * x * z, 2 * y * z]
        end

        # Set up the target values (1 for each point)
        target = Array.new(points.size, 1)

        # Solve the linear system using least squares (normal equations)
        design_matrix = Matrix[*design_matrix]
        target_vector = Vector.elements(target)

        coefficients = (design_matrix.transpose * design_matrix).inverse * design_matrix.transpose * target_vector
        coefficients.to_a
      end

      # Extract ellipsoid semi-axes and rotation matrix from the coefficients
      def self.ellipsoid_parameters(coefficients)
        a, b, c, d, e, f = coefficients

        # Construct the ellipsoid matrix from the coefficients
        ellipsoid_matrix = Matrix[
          [a, d, e],
          [d, b, f],
          [e, f, c]
        ]

        # Eigenvalue decomposition gives us the semi-axes and rotation matrix
        eigen = Matrix::EigenvalueDecomposition.new(ellipsoid_matrix)

        # Semi-axes lengths are the inverse of the square roots of the eigenvalues
        semi_axes_lengths = eigen.eigenvalues.map { |val| 1.0 / Math.sqrt(val) }

        # Rotation matrix is the matrix of eigenvectors
        rotation_matrix = eigen.eigenvector_matrix

        # Check the determinant of the rotation matrix
        if rotation_matrix.determinant < 0
          # If determinant is negative, flip one axis to correct the mirroring
          rotation_matrix = rotation_matrix.map.with_index do |value, index|
            # Negate the first column (or any one column) to flip the orientation
            index % 3 == 0 ? value : -value
          end
        end

        {
          a: semi_axes_lengths[0], # Semi-axis along x
          b: semi_axes_lengths[1], # Semi-axis along y
          c: semi_axes_lengths[2], # Semi-axis along z
          rotation_matrix: rotation_matrix
        }
      end

      def self.from_measurements(points)
        n = points.size
        min_x, min_y, min_z = Float::INFINITY, Float::INFINITY, Float::INFINITY
        max_x, max_y, max_z = -Float::INFINITY, -Float::INFINITY, -Float::INFINITY
      
        # Compute the average of x, y, z to find the offset (center of ellipse)
        points.each do |x, y, z|
          min_x = [x, min_x].min
          min_y = [y, min_y].min
          min_z = [z, min_z].min
          max_x = [x, max_x].max
          max_y = [y, max_y].max
          max_z = [z, max_z].max
        end

        offset_x = (min_x + max_x) / 2
        offset_y = (min_y + max_y) / 2
        offset_z = (min_z + max_z) / 2
      
        # Step 2: Correct for hard iron effect by shifting the points
        corrected_points = points.map do |x, y, z|
          [x - offset_x, y - offset_y, z - offset_z]
        end

        coeffs = fit_ellipsoid(corrected_points)
        params = ellipsoid_parameters(coeffs)

        offset = [offset_x, offset_y, offset_z]
        scaling = [params[:a], params[:b], params[:c]]
        rotation = params[:rotation_matrix]

        self.new(offset, scaling, rotation)
      end

      def apply(point)
        x = point[0] - @offset[0]
        y = point[1] - @offset[1]
        z = point[2] - @offset[2]

        # Apply the inverse rotation
        rotated_point = @rotation.transpose * Vector[x, y, z]
        
        # Scale the point to map the ellipsoid to a unit sphere
        [
          rotated_point[0] / @scaling[0],
          rotated_point[1] / @scaling[1],
          rotated_point[2] / @scaling[2]
        ]
      end
    end

    include Loggable

    attr_accessor :on_azimuth, :on_elevation, :calibration

    def initialize(config, data_dir, opts = {})
      @port = config['port'] || raise('port is required')
      @baud = config['baud'] || raise('baud is required')
      @declination = config['magnetic_declination'] || raise('magnetic_declination is required')
      @data_dir = data_dir
      @calibration = opts[:calibration] || Calibration.new

      @roll_degrees = 0.0
      @pitch_degrees = 0.0
      @yaw_degrees = 0.0

      @capture_mutex = Mutex.new
      @on_azimuth = nil
      @on_elevation = nil
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

    def start_raw_capture
      @capture_mutex.synchronize do
        return if @capturing

        @capturing = true
        @cal_data = []

        log 'Starting capture...'
      end
    end

    def end_raw_capture
      @capture_mutex.synchronize do
        return unless @capturing

        log 'Ended capture'
        @capturing = false
        @cal_data
      end
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
    
      @capture_mutex.synchronize do
        if @capturing
          @cal_data << [x, y, z]
          return
        end
      end

      x, y, z = @calibration.apply([x, y, z])
      
      # Sensor is placed with the mounting plate facing the ground, and the +Y axis pointing down the
      # boom of the antenna.

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
