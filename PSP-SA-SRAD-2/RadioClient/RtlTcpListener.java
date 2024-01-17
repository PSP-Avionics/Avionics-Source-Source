import java.io.*;
import java.net.*;
import java.util.*;

public class RtlTcpListener {
	public static class BitStream {
		private int numBits;
		private int currentByte;

		public BitStream() {
			numBits = 0;
			currentByte = 0;
		}

		public void addBit(int bit) {
			currentByte = (currentByte << 1) | (bit & 0x1);
			numBits++;
			if (numBits == 8) {
				reset();
			}
		}

		public static String toBinaryStringWithPadding(int value, int numBits) {
			String binaryString = Integer.toBinaryString(value);
			int numZerosToAdd = numBits - binaryString.length();
			if (numZerosToAdd > 0) {
				StringBuilder sb = new StringBuilder();
				for (int i = 0; i < numZerosToAdd; i++) {
				    sb.append('0');
				}
				sb.append(binaryString);
				return sb.toString();
			} else {
				return binaryString;
			}
		}

		public void reset() {
        	//System.out.print(toBinaryStringWithPadding(currentByte, numBits));
		    numBits = 0;
		    currentByte = 0;
		}
	}

	public static class Timer {
		private long periodNanos;
		private long lastTime;
		private int syncCount;
		private long lastPrintTime;

		public Timer(int eventsPerSecond) {
		    this.periodNanos = 1000000000L / eventsPerSecond;
		    this.lastTime = System.nanoTime();
		    this.syncCount = 0;
		    this.lastPrintTime = System.currentTimeMillis();
		}

		public void sync() throws InterruptedException {
		    long currentTime = System.nanoTime();
		    long elapsedTime = currentTime - lastTime;
		    long sleepTime = periodNanos - elapsedTime;
		    if (sleepTime > 0) {
		        long sleepMillis = sleepTime / 1000000L;
		        int sleepNanos = (int) (sleepTime % 1000000L);
		        Thread.sleep(sleepMillis, sleepNanos);
		    }
		    lastTime = System.nanoTime();
		    syncCount++;
		    long currentTimeMillis = System.currentTimeMillis();
		    if (currentTimeMillis - lastPrintTime >= 1000) {
//		        System.out.println("Sync count: " + syncCount);
		        syncCount = 0;
		        lastPrintTime = currentTimeMillis;
		    }
		}
	}



	/* FYI, you know this code was ChatGPT generated because I never leave comments, lol */
    public static void main(String[] args) throws Exception {
        String serverAddress = "localhost"; // replace with the IP address of your rtl_tcp server
        int serverPort = 8089; // replace with the port number of your rtl_tcp server
        int frequency1 = 445454000; // frequency for '0' bit
        int frequency2 = 445456000; // frequency for '1' bit
        int sampleRate = 32; // the number of samples per second
        int minSignalStrength = 10000; // the minimum signal strength required to consider a frequency
		int samplesPerBit = 1200;


		// freq 2 is the higher freq

		BitStream bs = new BitStream();
		Timer tim = new Timer(sampleRate);

        // connect to the rtl_tcp server
        Socket socket0 = new Socket(serverAddress, serverPort);
        OutputStream outputStream0 = socket0.getOutputStream();
        InputStream inputStream0 = socket0.getInputStream();

        Socket socket1 = new Socket(serverAddress, serverPort);
        OutputStream outputStream1 = socket1.getOutputStream();
        InputStream inputStream1 = socket1.getInputStream();

		send_message(outputStream0, inputStream0, (byte) 0, frequency1, 201600, (frequency1 + frequency2) / 2);
		send_message(outputStream1, inputStream1, (byte) 0, frequency2, 201600, (frequency1 + frequency2) / 2);

        // set the frequency and sample rate
//        String setFrequencyCommand1 = "SET_FREQ " + frequency1 + "\n";
//        outputStream0.write(setFrequencyCommand1.getBytes());
//        String setFrequencyCommand2 = "SET_FREQ " + frequency2 + "\n";
//        outputStream1.write(setFrequencyCommand2.getBytes());
//        String setFrequencyCommand2 = "F " + frequency2 + "\n";
//        outputStream.write(setFrequencyCommand2.getBytes());
//        String setSampleRateCommand = "s " + (sampleRate * samplesPerBit) + "\n";
//        outputStream.write(setSampleRateCommand.getBytes());
//        setSampleRateCommand = "b " + 1 + "\n";
//        outputStream.write(setSampleRateCommand.getBytes());
//        setSampleRateCommand = "n " + 1 + "\n";
//        outputStream.write(setSampleRateCommand.getBytes());

        // read the samples and determine which frequency has a higher signal
        int numSamples = sampleRate / 2; // read 1 second's worth of samples
        int numBytes = numSamples * 4; // 4 bytes per float
        byte[] buffer = new byte[numBytes];
        int numBytesRead;
        float sum1 = 0;
        float sum2 = 0;
		int samplesTaken = 0;
        boolean lowerFreqActive = false;
        boolean higherFreqActive = false;
		while (true) {
			sum1 = 0;
			sum2 = 0;
			samplesTaken = 0;
			lowerFreqActive = false;
			higherFreqActive = false;
			

			for (int i = 0; i < ((lowerFreqActive || higherFreqActive) ? 1 : samplesPerBit); i ++) {
//        		outputStream.write(setFrequencyCommand1.getBytes());
//			    outputStream.write("R\n".getBytes()); // request samples
//        		outputStream.write(setFrequencyCommand2.getBytes());
//			    outputStream.write("R\n".getBytes()); // request samples
			    numBytesRead = inputStream0.read(buffer, 0, numBytes);
				float[] sample = conv_to_float(buffer, numBytesRead);
				System.out.println("freq0"+Arrays.toString(sample));

				for (float f : sample) sum1 += f;
				
//				System.out.println("numBytes0: " + numBytes + " read: " + numBytesRead +" avail: " + inputStream0.available());
//				System.out.println(java.util.Arrays.toString(buffer));
//			    for (int j = 0; j < numBytesRead; j += 2) {
//			        int sample = (buffer[j] & 0xff) | ((buffer[j + 1] & 0xff) << 8);
//			        sum1 += sample;
//					System.out.println("sample0: " + sample + " " + sum1 + " " + sum2);
//			    }
				numBytesRead = inputStream1.read(buffer, 0, numBytes);
				sample = conv_to_float(buffer, numBytesRead);
				System.out.println("freq1: "+Arrays.toString(sample));

				for (float f : sample) sum2 += f;
				
				samplesTaken += sample.length;

//				System.out.println("numBytes1: " + numBytes + " read: " + numBytesRead +" avail: " + inputStream1.available());
//				System.out.println(java.util.Arrays.toString(buffer));
//			    for (int j = 0; j < numBytesRead; j += 2) {
//			        int sample = (buffer[j] & 0xff) | ((buffer[j + 1] & 0xff) << 8);
//			        sum2 += sample;
//					System.out.println("sample1: " + sample + " " + sum1 + " " + sum2);
//			    }

			}
	   	    float avg1 = sum1 / samplesTaken;
	        float avg2 = sum2 / samplesTaken;
			System.out.printf("avgs: %.4f\t%.4f\n", avg1, avg2);
			if (avg1 < minSignalStrength && avg2 < minSignalStrength) {
				// do nothing if neither frequency has a strong enough signal
			} else if (avg1 >= minSignalStrength && avg2 < minSignalStrength) {
					//System.out.println("Frequency " + frequency1 + " has a higher signal");
				bs.addBit(0);
			} else if (avg1 < minSignalStrength && avg2 >= minSignalStrength) {
					//System.out.println("Frequency " + frequency2 + " has a higher signal");
				bs.addBit(1);
			} else {
				if (avg1 > avg2) {
						//System.out.println("Frequency " + frequency1 + " has a higher signal");
					bs.addBit(0);
				} else if (avg1 < avg2) {
						//System.out.println("Frequency " + frequency2 + " has a higher signal");
					bs.addBit(1);
				} else {
						//System.out.println("Both frequencies have the same signal strength");
					bs.reset();
					System.out.println(" done");
				}
			}

		   tim.sync();
       }

    }

	private static void send_message(OutputStream sock_os, InputStream sock_is, byte type, int center_freq, int sampling_rate, int band_freq) throws IOException {
		byte[] buffer = new byte[2 + 13];
		buffer[0] = 0; // protocol ver
		buffer[1] = type; // 0 = request, 1 = shutdown, 3 = ping, 2 = repsonse
		write_int(buffer, 2, center_freq); // center-freq
		write_int(buffer, 6, sampling_rate); // sampling_rate
		write_int(buffer, 10, band_freq); // band-freq
		buffer[14] = 1; // dest (0 = file, 1 = socket)

		sock_os.write(buffer);

		sock_is.read(buffer, 0, 2 + 5);
		assert buffer[0] == 0; // protocol version
		assert buffer[1] == 2; // TYPE_RESPONSE
		assert buffer[2] == 0; // status = success
		// the rest of the buffer is a file idx, so i dont care
		//assert_response(client0, TYPE_RESPONSE, RESPONSE_STATUS_SUCCESS, 0);
	}

	private static void write_int(byte[] data, int off, int val) {
		data[off+0] = (byte) ((val >> 24) & 0xFF);	
		data[off+1] = (byte) ((val >> 16) & 0xFF);	
		data[off+2] = (byte) ((val >> 8) & 0xFF);	
		data[off+3] = (byte) ((val >> 0) & 0xFF);	
	}

	private static float[] conv_to_float(byte[] data, int len) {
		assert (len % 4 == 0);
		float[] dat = new float[len/4];
		for (int i = 0; i < dat.length; i ++) {
			int intBits = ((((int) data[3+i*4]) << 24) & 0xFF000000) | ((((int) data[2+i*4]) << 24) & 0xFF0000) | ((((int) data[1+i*4]) << 24) & 0xFF00) | ((((int) data[0+i*4]) << 24) & 0xFF);
			dat[i] = Float.intBitsToFloat(intBits);
		}	
		return dat;
	}
}



//int send_message(struct tcp_client *client, uint8_t protocol, uint8_t type, uint32_t center_freq, uint32_t sampling_rate, uint32_t band_freq, uint8_t destination) {
//	struct message_header header;
//	header.protocol_version = protocol;
//	header.type = type;
//	struct request req;
//	req.band_freq = band_freq;
//	req.center_freq = center_freq;
//	req.sampling_rate = sampling_rate;
//	req.destination = destination;
//	return write_request(header, req, client);
//}
