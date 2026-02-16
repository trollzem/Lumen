/**
 * @file src/platform/macos/sc_audio.m
 * @brief ScreenCaptureKit-based system audio capture implementation.
 * @details Captures system audio by starting a minimal screen capture stream
 *          with audio enabled. This allows capturing all system audio output.
 */
#import "sc_audio.h"

API_AVAILABLE(macos(12.3))
@implementation SCAudioCapture

+ (BOOL)isAvailable {
    if (@available(macOS 12.3, *)) {
        return YES;
    }
    return NO;
}

+ (NSArray<NSString *> *)availableAudioSources {
    // ScreenCaptureKit captures system audio, so we report that as available
    NSMutableArray *sources = [NSMutableArray array];

    if ([SCAudioCapture isAvailable]) {
        [sources addObject:@"System Audio (ScreenCaptureKit)"];
    }

    return sources;
}

- (instancetype)init {
    self = [super init];
    if (self) {
        self.isCapturing = NO;

        dispatch_queue_attr_t qos = dispatch_queue_attr_make_with_qos_class(
            DISPATCH_QUEUE_SERIAL, QOS_CLASS_USER_INITIATED, DISPATCH_QUEUE_PRIORITY_HIGH);
        self.audioQueue = dispatch_queue_create("dev.lizardbyte.sunshine.scAudioQueue", qos);

        self.samplesArrivedSignal = [[NSCondition alloc] init];
    }
    return self;
}

- (TPCircularBuffer *)getAudioBuffer {
    return &_audioSampleBuffer;
}

- (void)dealloc {
    [self stopCapture];
    [self.samplesArrivedSignal release];
    [super dealloc];
}

- (int)startCaptureWithSampleRate:(UInt32)sampleRate channels:(UInt8)channels {
    if (self.isCapturing) {
        NSLog(@"[SCAudioCapture] Already capturing");
        return -1;
    }

    // Initialize the circular buffer
    TPCircularBufferInit(&_audioSampleBuffer, kAudioBufferLength);

    // Get shareable content
    dispatch_semaphore_t initSemaphore = dispatch_semaphore_create(0);
    __block BOOL initSuccess = NO;

    [SCShareableContent getShareableContentWithCompletionHandler:^(SCShareableContent *content, NSError *error) {
        if (error) {
            NSLog(@"[SCAudioCapture] Failed to get shareable content: %@", error.localizedDescription);
        } else {
            self.shareableContent = content;
            initSuccess = YES;
        }
        dispatch_semaphore_signal(initSemaphore);
    }];

    long result = dispatch_semaphore_wait(initSemaphore, dispatch_time(DISPATCH_TIME_NOW, 5 * NSEC_PER_SEC));
    if (result != 0 || !initSuccess) {
        NSLog(@"[SCAudioCapture] Timeout or failure getting shareable content");
        return -1;
    }

    // We need at least one display to create a content filter
    if (self.shareableContent.displays.count == 0) {
        NSLog(@"[SCAudioCapture] No displays available");
        return -1;
    }

    SCDisplay *display = self.shareableContent.displays.firstObject;
    SCContentFilter *filter = [[SCContentFilter alloc] initWithDisplay:display excludingWindows:@[]];

    // Configure the stream - we want audio, minimal video
    SCStreamConfiguration *config = [[SCStreamConfiguration alloc] init];

    // Minimal video settings (required, but we don't use the video)
    config.width = 64;
    config.height = 64;
    config.minimumFrameInterval = CMTimeMake(1, 1);  // 1 fps - minimum possible
    config.pixelFormat = kCVPixelFormatType_32BGRA;
    config.queueDepth = 1;
    config.showsCursor = NO;

    // Audio settings - this is what we want!
    config.capturesAudio = YES;
    config.excludesCurrentProcessAudio = YES;  // Don't capture our own audio output
    config.sampleRate = sampleRate;
    config.channelCount = channels;

    // Create the stream
    NSError *error = nil;
    self.stream = [[SCStream alloc] initWithFilter:filter configuration:config delegate:self];

    if (!self.stream) {
        NSLog(@"[SCAudioCapture] Failed to create SCStream");
        return -1;
    }

    // Add audio output - this is the key!
    if (![self.stream addStreamOutput:self type:SCStreamOutputTypeAudio sampleHandlerQueue:self.audioQueue error:&error]) {
        NSLog(@"[SCAudioCapture] Failed to add audio output: %@", error.localizedDescription);
        return -1;
    }

    // We don't need video output, but we need to add screen output for the stream to work
    // However, we can skip this and just let the stream run without processing video

    // Start the capture
    dispatch_semaphore_t startSemaphore = dispatch_semaphore_create(0);
    __block BOOL startSuccess = NO;

    [self.stream startCaptureWithCompletionHandler:^(NSError *captureError) {
        if (captureError) {
            NSLog(@"[SCAudioCapture] Failed to start capture: %@", captureError.localizedDescription);
        } else {
            NSLog(@"[SCAudioCapture] System audio capture started successfully!");
            startSuccess = YES;
        }
        dispatch_semaphore_signal(startSemaphore);
    }];

    result = dispatch_semaphore_wait(startSemaphore, dispatch_time(DISPATCH_TIME_NOW, 5 * NSEC_PER_SEC));
    if (result != 0 || !startSuccess) {
        NSLog(@"[SCAudioCapture] Timeout or failure starting capture");
        return -1;
    }

    self.isCapturing = YES;
    return 0;
}

- (void)stopCapture {
    if (!self.isCapturing) {
        return;
    }

    self.isCapturing = NO;

    if (self.stream) {
        dispatch_semaphore_t stopSemaphore = dispatch_semaphore_create(0);

        [self.stream stopCaptureWithCompletionHandler:^(NSError *error) {
            if (error) {
                NSLog(@"[SCAudioCapture] Error stopping capture: %@", error.localizedDescription);
            }
            dispatch_semaphore_signal(stopSemaphore);
        }];

        dispatch_semaphore_wait(stopSemaphore, dispatch_time(DISPATCH_TIME_NOW, 2 * NSEC_PER_SEC));
        self.stream = nil;
    }

    // Signal any waiting threads
    [self.samplesArrivedSignal signal];

    // Clean up the buffer
    TPCircularBufferCleanup(&_audioSampleBuffer);
}

#pragma mark - SCStreamDelegate

- (void)stream:(SCStream *)stream didStopWithError:(NSError *)error {
    NSLog(@"[SCAudioCapture] Stream stopped with error: %@", error.localizedDescription);
    self.isCapturing = NO;
    [self.samplesArrivedSignal signal];
}

#pragma mark - SCStreamOutput

- (void)stream:(SCStream *)stream
    didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer
                   ofType:(SCStreamOutputType)type {

    if (type == SCStreamOutputTypeAudio) {
        // Get the format description to understand the audio format
        CMFormatDescriptionRef formatDesc = CMSampleBufferGetFormatDescription(sampleBuffer);
        if (!formatDesc) {
            return;
        }

        const AudioStreamBasicDescription *asbd = CMAudioFormatDescriptionGetStreamBasicDescription(formatDesc);
        if (!asbd) {
            return;
        }

        // Log format on first sample (for debugging)
        static BOOL formatLogged = NO;
        if (!formatLogged) {
            NSLog(@"[SCAudioCapture] Audio format: %.0f Hz, %d channels, %d bits, format flags: 0x%x, non-interleaved: %d",
                  asbd->mSampleRate, asbd->mChannelsPerFrame, asbd->mBitsPerChannel, asbd->mFormatFlags,
                  (asbd->mFormatFlags & kAudioFormatFlagIsNonInterleaved) != 0);
            formatLogged = YES;
        }

        BOOL isFloat = (asbd->mFormatFlags & kAudioFormatFlagIsFloat) != 0;
        BOOL isNonInterleaved = (asbd->mFormatFlags & kAudioFormatFlagIsNonInterleaved) != 0;

        // For non-interleaved audio, we need a larger buffer to hold all channels
        // Calculate the required buffer size
        size_t bufferListSize = sizeof(AudioBufferList);
        if (isNonInterleaved && asbd->mChannelsPerFrame > 1) {
            // Need space for multiple AudioBuffer structs
            bufferListSize = sizeof(AudioBufferList) + (asbd->mChannelsPerFrame - 1) * sizeof(AudioBuffer);
        }

        // Allocate the buffer list
        AudioBufferList *audioBufferList = (AudioBufferList *)malloc(bufferListSize);
        if (!audioBufferList) {
            return;
        }

        CMBlockBufferRef blockBuffer = NULL;

        OSStatus status = CMSampleBufferGetAudioBufferListWithRetainedBlockBuffer(
            sampleBuffer,
            &bufferListSize,
            audioBufferList,
            bufferListSize,
            NULL,
            NULL,
            kCMSampleBufferFlag_AudioBufferList_Assure16ByteAlignment,
            &blockBuffer
        );

        if (status == noErr && audioBufferList->mNumberBuffers > 0) {
            if (isFloat && asbd->mBitsPerChannel == 32) {
                if (isNonInterleaved && audioBufferList->mNumberBuffers >= 2) {
                    // Non-interleaved stereo: need to interleave L and R channels
                    // Buffer 0 = Left channel (L, L, L, ...)
                    // Buffer 1 = Right channel (R, R, R, ...)
                    // Output should be: (L, R, L, R, L, R, ...)

                    float *leftChannel = (float *)audioBufferList->mBuffers[0].mData;
                    float *rightChannel = (float *)audioBufferList->mBuffers[1].mData;
                    size_t samplesPerChannel = audioBufferList->mBuffers[0].mDataByteSize / sizeof(float);

                    // Allocate interleaved buffer
                    size_t interleavedSize = samplesPerChannel * 2 * sizeof(float);
                    float *interleavedData = (float *)malloc(interleavedSize);

                    if (interleavedData && leftChannel && rightChannel) {
                        for (size_t i = 0; i < samplesPerChannel; i++) {
                            interleavedData[i * 2] = leftChannel[i];      // Left
                            interleavedData[i * 2 + 1] = rightChannel[i]; // Right
                        }
                        TPCircularBufferProduceBytes(&_audioSampleBuffer, interleavedData, interleavedSize);
                        free(interleavedData);
                    }
                } else {
                    // Already interleaved Float32 - write directly
                    AudioBuffer audioBuffer = audioBufferList->mBuffers[0];
                    if (audioBuffer.mData && audioBuffer.mDataByteSize > 0) {
                        TPCircularBufferProduceBytes(&_audioSampleBuffer, audioBuffer.mData, audioBuffer.mDataByteSize);
                    }
                }

                // Signal that samples have arrived
                [self.samplesArrivedSignal signal];
            } else {
                // Unsupported format
                static BOOL unknownFormatLogged = NO;
                if (!unknownFormatLogged) {
                    NSLog(@"[SCAudioCapture] Unsupported audio format: %d bits, float: %d, flags: 0x%x",
                          asbd->mBitsPerChannel, isFloat, asbd->mFormatFlags);
                    unknownFormatLogged = YES;
                }
            }
        }

        free(audioBufferList);

        if (blockBuffer) {
            CFRelease(blockBuffer);
        }
    }
    // We ignore video frames (type == SCStreamOutputTypeScreen)
}

@end
