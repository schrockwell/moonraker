class IO
  # For serial components
  def read_next_string_command(terminator)
    buffer = ''

    while true do
      next if eof?
      chr = readchar

      buffer << chr
      if chr == terminator
        return buffer
      end
    end
  end
end