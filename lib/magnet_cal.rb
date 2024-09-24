module Moonraker
  class MagnetCal
    def initialize(imu_config, data_dir)
      @port = imu_config['port'] || raise 'port is required'
      @baud = imu_config['baud'] || raise 'baud is required'
      @data_dir = data_dir
    end

    def run
      
    end
  end
end