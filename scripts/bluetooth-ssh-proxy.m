// SSH byte transport over a private RFCOMM service, without macOS virtual TTYs.
#import <Foundation/Foundation.h>
#import <IOBluetooth/IOBluetooth.h>
#include <errno.h>
#include <signal.h>
#include <unistd.h>

@interface BluetoothSSH : NSObject <IOBluetoothRFCOMMChannelDelegate>
@property(strong) IOBluetoothDevice *device;
@property(strong) IOBluetoothRFCOMMChannel *channel;
@property(strong) NSData *pending;
@property(strong) dispatch_semaphore_t sent;
@property BOOL probe;
@property BOOL ready;
@property BOOL closing;
- (void)readInput;
- (void)openChannel;
- (void)finish:(int)code;
@end

@implementation BluetoothSSH
- (void)finish:(int)code {
    dispatch_async(dispatch_get_main_queue(), ^{
        if (self.closing) return;
        self.closing = YES;
        [self.channel setDelegate:nil];
        [self.channel closeChannel];
        [self.device closeConnection];
        // Give the system's asynchronous channel teardown time to finish.
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, NSEC_PER_SEC), dispatch_get_main_queue(), ^{ exit(code); });
    });
}
- (void)openChannel {
    IOBluetoothRFCOMMChannel *channel = nil;
    IOReturn status = [self.device openRFCOMMChannelAsync:&channel withChannelID:23 delegate:self];
    self.channel = channel;
    [channel setDelegate:self];
    fprintf(stderr, "Bluetooth SSH open requested: 0x%x, channel=%s\n", status, channel ? "created" : "none");
    if (status) {
        fprintf(stderr, "Bluetooth SSH open failed: 0x%x\n", status);
        exit(1);
    }
}
- (void)rfcommChannelOpenComplete:(IOBluetoothRFCOMMChannel *)channel status:(IOReturn)status {
    if (status) {
        fprintf(stderr, "Bluetooth SSH connection failed: 0x%x\n", status);
        exit(1);
    }
    self.channel = channel;
    self.ready = YES;
    fprintf(stderr, "Bluetooth SSH connected (MTU %u)\n", [channel getMTU]);
    if (!self.probe) [NSThread detachNewThreadSelector:@selector(readInput) toTarget:self withObject:nil];
}
- (void)rfcommChannelData:(IOBluetoothRFCOMMChannel *)channel data:(void *)bytes length:(size_t)length {
    const char *cursor = bytes;
    size_t remaining = length;
    while (remaining) {
        ssize_t count = write(STDOUT_FILENO, cursor, remaining);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) exit(1);
        cursor += count;
        remaining -= count;
    }
    if (self.probe && memchr(bytes, '\n', length)) {
        [self finish:0];
    }
}
- (void)rfcommChannelClosed:(IOBluetoothRFCOMMChannel *)channel { [self finish:0]; }
- (void)rfcommChannelWriteComplete:(IOBluetoothRFCOMMChannel *)channel refcon:(void *)refcon status:(IOReturn)status {
    if (status) {
        fprintf(stderr, "Bluetooth SSH write failed: 0x%x\n", status);
        exit(1);
    }
    self.pending = nil;
    dispatch_semaphore_signal(self.sent);
}
- (void)readInput {
    char buffer[65535];
    size_t mtu = [self.channel getMTU];
    for (;;) {
        @autoreleasepool {
            ssize_t count = read(STDIN_FILENO, buffer, mtu);
            if (count < 0 && errno == EINTR) continue;
            if (count <= 0) {
                [self finish:0];
                return;
            }
            NSData *data = [NSData dataWithBytes:buffer length:count];
            dispatch_async(dispatch_get_main_queue(), ^{
                self.pending = data;
                IOReturn status = [self.channel writeAsync:(void *)data.bytes length:(UInt16)data.length refcon:NULL];
                if (status) { fprintf(stderr, "Bluetooth SSH send failed: 0x%x\n", status); exit(1); }
            });
            dispatch_semaphore_wait(self.sent, DISPATCH_TIME_FOREVER);
        }
    }
}
@end

int main(int argc, const char **argv) {
    @autoreleasepool {
        if (argc < 2 || argc > 3) { fprintf(stderr, "usage: bluetooth-ssh-proxy ADDRESS [--probe]\n"); return 2; }
        signal(SIGPIPE, SIG_IGN);
        BluetoothSSH *proxy = [BluetoothSSH new];
        proxy.sent = dispatch_semaphore_create(0);
        signal(SIGTERM, SIG_IGN);
        signal(SIGINT, SIG_IGN);
        dispatch_source_t terminate = dispatch_source_create(DISPATCH_SOURCE_TYPE_SIGNAL, SIGTERM, 0, dispatch_get_main_queue());
        dispatch_source_t interrupt = dispatch_source_create(DISPATCH_SOURCE_TYPE_SIGNAL, SIGINT, 0, dispatch_get_main_queue());
        dispatch_source_set_event_handler(terminate, ^{ [proxy finish:0]; });
        dispatch_source_set_event_handler(interrupt, ^{ [proxy finish:0]; });
        dispatch_resume(terminate);
        dispatch_resume(interrupt);
        proxy.probe = argc == 3 && !strcmp(argv[2], "--probe");
        proxy.device = [IOBluetoothDevice deviceWithAddressString:@(argv[1])];
        if (!proxy.device || ![proxy.device isPaired]) { fprintf(stderr, "Pair XT-STCAR first\n"); return 1; }
        IOReturn status = [proxy.device isConnected] ? kIOReturnSuccess : [proxy.device openConnection];
        if (status) { fprintf(stderr, "Bluetooth base link failed: 0x%x\n", status); return 1; }
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0), ^{ [proxy openChannel]; });
        dispatch_after(dispatch_time(DISPATCH_TIME_NOW, 20*NSEC_PER_SEC), dispatch_get_main_queue(), ^{
            if (!proxy.ready) { fprintf(stderr, "Bluetooth SSH connection timed out\n"); exit(1); }
        });
        CFRunLoopRun();
    }
    return 0;
}
