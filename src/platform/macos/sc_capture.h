/**
 * @file src/platform/macos/sc_capture.h
 * @brief Declarations for ScreenCaptureKit-based capture on macOS 12.3+.
 * @details This implementation captures both video AND system audio using ScreenCaptureKit.
 */
#pragma once

#import <AppKit/AppKit.h>
#import <ScreenCaptureKit/ScreenCaptureKit.h>
#import <CoreMedia/CoreMedia.h>

API_AVAILABLE(macos(12.3))
@interface SCCapture : NSObject <SCStreamDelegate, SCStreamOutput>

#define kMaxDisplays 32

@property (nonatomic, assign) CGDirectDisplayID displayID;
@property (nonatomic, assign) int frameRate;
@property (nonatomic, assign) OSType pixelFormat;
@property (nonatomic, assign) int frameWidth;
@property (nonatomic, assign) int frameHeight;
@property (nonatomic, assign) BOOL captureAudio;

@property (nonatomic, strong) SCStream *stream;
@property (nonatomic, strong) SCShareableContent *shareableContent;
@property (nonatomic, strong) dispatch_queue_t videoQueue;
@property (nonatomic, strong) dispatch_queue_t audioQueue;

typedef bool (^VideoFrameCallbackBlock)(CMSampleBufferRef);
typedef void (^AudioSampleCallbackBlock)(CMSampleBufferRef);

@property (nonatomic, copy) VideoFrameCallbackBlock videoCallback;
@property (nonatomic, copy) AudioSampleCallbackBlock audioCallback;
@property (nonatomic, strong) dispatch_semaphore_t captureSignal;
@property (nonatomic, assign) BOOL stopping;
@property (nonatomic, assign) CMSampleBufferRef lastValidSampleBuffer;
@property (nonatomic, strong) dispatch_source_t frameDeliveryTimer;  // 60Hz timer for steady frame delivery
@property (nonatomic, strong) dispatch_queue_t deliveryQueue;        // queue for the timer

+ (NSArray<NSDictionary *> *)displayNames;
+ (NSString *)getDisplayName:(CGDirectDisplayID)displayID;
+ (BOOL)isAvailable;

- (instancetype)initWithDisplay:(CGDirectDisplayID)displayID
                      frameRate:(int)frameRate
                   captureAudio:(BOOL)captureAudio;

- (void)setFrameWidth:(int)frameWidth frameHeight:(int)frameHeight;
- (dispatch_semaphore_t)captureVideo:(VideoFrameCallbackBlock)videoCallback
                         audioCallback:(AudioSampleCallbackBlock)audioCallback;
- (void)stopCapture;

@end
