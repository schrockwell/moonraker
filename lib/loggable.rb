module Loggable
  def log(msg)
    puts "[#{Time.now.utc}] #{msg}"
  end
end