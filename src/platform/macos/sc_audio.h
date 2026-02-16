/**
 * @file src/platform/macos/sc_audio.h
 * @brief ScreenCaptureKit-based system audio capture for macOS 12.3+.
 * @details Captures system audio using ScreenCaptureKit API.
 */
#pragma once

#import <AppKit/AppKit.h>
#import <ScreenCaptureKit/ScreenCaptureKit.h>
#import <CoreMedia/CoreMedia.h>
#import <AudioToolbox/AudioToolbox.h>
#include "third-party/TPCircularBuffer/TPCircularBuffer.h"

// Buffer size for audio samples (in bytes)
#define kAudioBufferLength (48000 * 4 * 2 * 2)  // 2 seconds of stereo float audio at 48kHz

API_AVAILABLE(macos(12.3))
@interface SCAudioCapture : NSObject <SCStreamDelegate, SCStreamOutput>

@property (nonatomic, strong) SCStream *stream;
@property (nonatomic, strong) SCShareableContent *shareableContent;
@property (nonatomic, strong) dispatch_queue_t audioQueue;
@property (nonatomic, strong) NSCondition *samplesArrivedSignal;
@property (nonatomic, assign) BOOL isCapturing;

// Audio sample buffer (circular buffer for thread-safe access)
// Use getAudioBuffer to get a pointer to this buffer
@property (nonatomic, assign) TPCircularBuffer audioSampleBuffer;

+ (BOOL)isAvailable;

// Returns a pointer to the internal audio buffer
- (TPCircularBuffer *)getAudioBuffer;
+ (NSArray<NSString *> *)availableAudioSources;

- (instancetype)init;
- (int)startCaptureWithSampleRate:(UInt32)sampleRate channels:(UInt8)channels;
- (void)stopCapture;

@end
