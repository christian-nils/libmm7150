#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <signal.h>
#include <stdio.h>
#include <errno.h>
#include <getopt.h>
#include <iio.h>

/* Accelerometer name */
static char *name = "accel_3d"; //FIXME
static int buffer_length = 1;
static int count = -1;

/* IIO structs required for streaming */
static struct iio_context *ctx;
// Streaming device
static struct iio_device *dev;
static struct iio_buffer  *rxbuf;
static struct iio_channel **channels;
static int channel_count;

static bool stop;
static bool has_repeat;

/* cleanup and exit */
static void shutdown()
{
	if (channels) { free(channels); }

	printf("* Destroying buffers\n");
	if (rxbuf) { iio_buffer_destroy(rxbuf); }

	printf("* Disassociate trigger\n");
	if (dev) { iio_device_set_trigger(dev, NULL); }

	printf("* Destroying context\n");
	if (ctx) { iio_context_destroy(ctx); }
	exit(0);
}

static void handle_sig(int sig)
{
	printf("Waiting for process to finish...\n");
	stop = true;
}

/* simple configuration and streaming */
int main(int argc, char **argv)
{
	// Hardware trigger
	struct iio_device *trigger;

	// Initialize variables
	int i,j,sample;

	// Listen to ctrl+c and assert
	signal(SIGINT, handle_sig);

	printf("* Acquiring IIO context\n");
	// Create the context, if it fails the program will terminate
	assert((ctx = iio_create_default_context()) && "No context");
	// Counts how many devices are present in the context, if none the program will terminate
	assert(iio_context_get_devices_count(ctx) > 0 && "No devices");

	printf("* Acquiring device %s\n", name);
	// Find and assign the device thanks to its name
	dev = iio_context_find_device(ctx, name);
	if (!dev) {
		perror("No device found");
		shutdown();
	}

	// Initialization of the channels corresponding to the device to allocate required memory
	printf("* Initializing IIO streaming channels:\n");
	for (i = 0; i < iio_device_get_channels_count(dev); ++i) {
		struct iio_channel *chn = iio_device_get_channel(dev, i);
		if (iio_channel_is_scan_element(chn)) {
			printf("%s\n", iio_channel_get_id(chn));
			channel_count++;
		}
	}
	// If no scan elements, the program will terminate
	if (channel_count == 0) {
		printf("No scan elements found\n");
		shutdown();
	}

	// allocate memory to the channels
	channels = calloc(channel_count, sizeof *channels);
	if (!channels) {
		perror("Channel array allocation failed");
		shutdown();
	}

	// Populate the allocated memory
	for (i = 0; i < channel_count; ++i) {
		struct iio_channel *chn = iio_device_get_channel(dev, i);
		if (iio_channel_is_scan_element(chn))
			channels[i] = chn;
	}

	// Enable the channels for buffered capture
	printf("* Enabling IIO streaming channels for buffered capture\n");
	for (i = 0; i < channel_count; ++i)
		iio_channel_enable(channels[i]);

	// Create a buffer with one sample
	printf("* Creating non-cyclic IIO buffers with %d samples\n", buffer_length);
	rxbuf = iio_device_create_buffer(dev, buffer_length, false);
	if (!rxbuf) {
		perror("Could not create buffer");
		shutdown();
	}

	// Starting the streaming
	printf("* Starting IO streaming (press CTRL+C to cancel)\n");
	bool has_ts = strcmp(iio_channel_get_id(channels[channel_count - 1]), "timestamp") == 0;
	int64_t last_ts = 0;
	while (!stop)
	{
		ssize_t nbytes_rx;
		void *p_dat, *p_end;
		ptrdiff_t p_inc;
		int64_t now_ts;

		// Refill RX buffer
		nbytes_rx = iio_buffer_refill(rxbuf);
		if (nbytes_rx < 0) {
			printf("Error refilling buf: %d\n", (int)nbytes_rx);
			shutdown();
		}

		p_inc = iio_buffer_step(rxbuf);
		p_end = iio_buffer_end(rxbuf);

		// Print timestamp delta in ms
		if (has_ts)
			for (p_dat = iio_buffer_first(rxbuf, channels[channel_count - 1]); p_dat < p_end; p_dat += p_inc) {
				now_ts = (((int64_t *)p_dat)[0]);
				printf("[%04ld] ", last_ts > 0 ? (now_ts - last_ts) / 1000 / 1000 : 0);
				last_ts = now_ts;
			}

		// Print each captured sample
		for (i = 0; i < channel_count; ++i) {
			uint8_t *buf;
			size_t bytes;
			const struct iio_data_format *fmt = iio_channel_get_data_format(channels[i]);
			unsigned int repeat = has_repeat ? fmt->repeat : 1;
			size_t sample_size = fmt->length / 8 * repeat;

			buf = malloc(sample_size * buffer_length);

			bytes = iio_channel_read(channels[i], rxbuf, buf, sample_size * buffer_length);

			printf("%s ", iio_channel_get_id(channels[i]));
			for (sample = 0; sample < bytes / sample_size; ++sample) {
				for (j = 0; j < repeat; ++j) {
					if (fmt->length / 8 == sizeof(int16_t))
						printf("%i ", ((int16_t *)buf)[sample + j]);
					else if (fmt->length / 8 == sizeof(int64_t))
						printf("%li ", ((int64_t *)buf)[sample + j]);
				}
			}

			free(buf);
		}
		printf("\n");

	}

	shutdown();

	return 0;

}
