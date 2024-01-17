import javax.sound.sampled.*;
import java.util.ArrayList;
import java.util.List;

public class AfskDecoder {
    private static final int MARK_FREQ = 1200;
    private static final int SPACE_FREQ = 2200;
    private static final int BAUD_RATE = 1200;
    private static final int SAMPLE_RATE = 44100;
    private static final int SAMPLE_SIZE_IN_BITS = 16;
    private static final int CHANNELS = 1;
    private static final boolean SIGNED = true;
    private static final boolean BIG_ENDIAN = false;
    private static final int BUFFER_SIZE = 1024;

    private static final int STATE_IDLE = 0;
    private static final int STATE_MARK = 1;
    private static final int STATE_SPACE = 2;

    private static final int MIN_SIGNAL_LENGTH = 10;
    private static final int MAX_SIGNAL_LENGTH = 10000;

    private static final int MIN_MARK_LENGTH = 50;
    private static final int MAX_MARK_LENGTH = 200;

    private static final int MIN_SPACE_LENGTH = 50;
    private static final int MAX_SPACE_LENGTH = 200;

    private static final int MIN_SIGNALS_PER_BYTE = 5;
    private static final int MAX_SIGNALS_PER_BYTE = 7;

    private static final int MIN_SIGNALS_PER_FRAME = 10;
    private static final int MAX_SIGNALS_PER_FRAME = 100;

    private static final int MIN_FRAME_LENGTH = 10;
    private static final int MAX_FRAME_LENGTH = 1000;

    private static final int MIN_FRAME_GAP = 10;
    private static final int MAX_FRAME_GAP = 1000;

    private static final int MIN_FRAME_SYNC = 10;
    private static final int MAX_FRAME_SYNC = 100;

    private static final int FRAME_SYNC_PATTERN = 0x7E;

    private static final int[] CRC_TABLE = {
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
            0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF
    };

    private static int crc16(byte[] data, int length) {
        int crc = 0xFFFF;
        for (int i = 0; i < length; i++) {
            crc = (crc << 4) ^ CRC_TABLE[((crc >> 12) ^ (data[i] >> 4)) & 0x0F];
            crc = (crc << 4) ^ CRC_TABLE[((crc >> 12) ^ (data[i] & 0x0F)) & 0x0F];
        }
        return crc;
    }

    public static void main(String[] args) {
        AudioFormat format = new AudioFormat(SAMPLE_RATE, SAMPLE_SIZE_IN_BITS, CHANNELS, SIGNED, BIG_ENDIAN);
        DataLine.Info info = new DataLine.Info(TargetDataLine.class, format);
        if (!AudioSystem.isLineSupported(info)) {
            System.err.println("Line not supported");
            System.exit(1);
        }

        TargetDataLine line;
        try {
            line = (TargetDataLine) AudioSystem.getLine(info);
            line.open(format);
        } catch (LineUnavailableException e) {
            System.err.println("Line unavailable");
            System.exit(1);
            return;
        }

        byte[] buffer = new byte[BUFFER_SIZE];
        List<Integer> signal = new ArrayList<>();
        int state = STATE_IDLE;
        int signal_length = 0;
        int mark_length = 0;
        int space_length = 0;
        int signals_per_byte = 0;
        int signals_per_frame = 0;
        int frame_length = 0;
        int frame_gap = 0;
        int frame_sync = 0;
        int frame_sync_count = 0;
        int frame_crc = 0;
        int frame_crc_count = 0;
        int frame_data_count = 0;
        byte[] frame_data = new byte[MAX_FRAME_LENGTH];
        int frame_data_index = 0;

        line.start();
        while (true) {
            int count = line.read(buffer, 0, buffer.length);
            for (int i = 0; i < count; i++) {
                int sample = buffer[i];
                signal.add(sample);

                if (state == STATE_IDLE) {
                    if (sample > 0) {
                        state = STATE_MARK;
                        signal_length = 1;
                        mark_length = 1;
                        space_length = 0;
                    }
                } else if (state == STATE_MARK) {
                    if (sample > 0) {
                        signal_length++;
                        mark_length++;
                    } else {
                        state = STATE_SPACE;
                        signal_length = 1;
                        space_length = 1;
                    }
                } else if (state == STATE_SPACE) {
                    if (sample > 0) {
                        state = STATE_MARK;
                        signal_length = 1;
                        mark_length = 1;
                        space_length = 0;
                    } else {
                        signal_length++;
                        space_length++;
                    }
                }

                if (signal_length >= MIN_SIGNAL_LENGTH && signal_length <= MAX_SIGNAL_LENGTH) {
                    if (state == STATE_MARK && mark_length >= MIN_MARK_LENGTH && mark_length <= MAX_MARK_LENGTH) {
                        int freq = (int) ((float) SAMPLE_RATE / signal_length);
                        if (freq == MARK_FREQ) {
                            signals_per_byte++;
                        } else if (freq == SPACE_FREQ) {
                            signals_per_byte <<= 1;
                        } else {
                            signals_per_byte = 0;
                        }
                    } else if (state == STATE_SPACE && space_length >= MIN_SPACE_LENGTH && space_length <= MAX_SPACE_LENGTH) {
                        int freq = (int) ((float) SAMPLE_RATE / signal_length);
                        if (freq == MARK_FREQ) {
                            signals_per_byte <<= 1;
                        } else if (freq == SPACE_FREQ) {
                            signals_per_byte++;
                        } else {
                            signals_per_byte = 0;
                        }
                    }

                    if (signals_per_byte >= MIN_SIGNALS_PER_BYTE && signals_per_byte <= MAX_SIGNALS_PER_BYTE) {
                        signals_per_frame++;
                        frame_length += signal_length;

                        if (signals_per_frame >= MIN_SIGNALS_PER_FRAME && signals_per_frame <= MAX_SIGNALS_PER_FRAME) {
                            if (frame_length >= MIN_FRAME_LENGTH && frame_length <= MAX_FRAME_LENGTH) {
                                if (frame_gap >= MIN_FRAME_GAP && frame_gap <= MAX_FRAME_GAP) {
                                    if (frame_sync >= MIN_FRAME_SYNC && frame_sync <= MAX_FRAME_SYNC) {
                                        if (frame_sync_count == 0 && signal_length >= MIN_MARK_LENGTH && signal_length <= MAX_MARK_LENGTH) {
                                            int freq = (int) ((float) SAMPLE_RATE / signal_length);
                                            if (freq == MARK_FREQ) {
                                                frame_sync_count = 1;
                                            }
                                        } else if (frame_sync_count > 0 && signal_length >= MIN_SPACE_LENGTH && signal_length <= MAX_SPACE_LENGTH) {
                                            int freq = (int) ((float) SAMPLE_RATE / signal_length);
                                            if (freq == SPACE_FREQ) {
                                                frame_sync_count++;
                                                if (frame_sync_count == 8) {
                                                    frame_sync_count = 0;
                                                    frame_crc_count = 0;
                                                    frame_data_count = 0;
                                                    frame_data_index = 0;
                                                }
                                            } else {
                                                frame_sync_count = 0;
                                            }
                                        }

                                        if (frame_sync_count == 0) {
                                            if (frame_crc_count == 0) {
                                                frame_crc = (signal_length << 8) & 0xFF00;
                                            } else if (frame_crc_count == 1) {
                                                frame_crc |= signal_length & 0xFF;
                                            } else {
                                                frame_data[frame_data_index++] = (byte) signal_length;
                                                frame_data_count++;
                                            }

                                            frame_crc_count++;
                                            if (frame_crc_count == 2) {
                                                frame_crc_count = 0;
                                            }

                                            if (frame_data_count == frame_length - 4) {
                                                int crc = crc16(frame_data, frame_data_count);
                                                if (crc == frame_crc) {
                                                    byte[] data = new byte[frame_data_count];
                                                    System.arraycopy(frame_data, 0, data, 0, frame_data_count);
                                                    System.out.println("Decoded data: " + new String(data));
                                                }
                                                signals_per_frame = 0;
                                                frame_length = 0;
                                                frame_gap = 0;
                                                frame_sync = 0;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if (state != STATE_IDLE) {
                    frame_gap += signal_length;
                }

                if (signal.size() > MAX_FRAME_LENGTH * 2) {
                    signal.remove(0);
                }
            }
        }
    }
}
