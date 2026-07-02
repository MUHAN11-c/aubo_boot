// USD Viewer error handler wrapper
// Capture and forward all errors to parent window

(function() {
    console.log('[USD Error Handler] Error handler loaded');

    // Capture all errors
    window.addEventListener('error', (event) => {
        console.error('[USD Error Handler] Error captured:', event.error || event.message);
        try {
            parent.postMessage({
                type: 'USD_ERROR',
                error: event.error?.message || event.message || 'Unknown error'
            }, '*');
        } catch (e) {
            console.error('[USD Error Handler] Failed to send error message:', e);
        }
    }, true);

    // Capture Promise rejections
    window.addEventListener('unhandledrejection', (event) => {
        console.error('[USD Error Handler] Promise rejection:', event.reason);
        try {
            parent.postMessage({
                type: 'USD_ERROR',
                error: event.reason?.message || String(event.reason) || 'Promise rejection'
            }, '*');
        } catch (e) {
            console.error('[USD Error Handler] Failed to send error message:', e);
        }
    });

    // Wrap original message handler, add error handling
    const originalAddEventListener = window.addEventListener;
    window.addEventListener = function(type, listener, ...args) {
        if (type === 'message') {
            const wrappedListener = async function(event) {
                try {
                    await listener.call(this, event);
                } catch (e) {
                    console.error('[USD Error Handler] Message handler error:', e);
                    try {
                        parent.postMessage({
                            type: 'USD_ERROR',
                            error: e.message || 'Message processing failed'
                        }, '*');
                    } catch (err) {
                        console.error('[USD Error Handler] Failed to send error message:', err);
                    }
                }
            };
            return originalAddEventListener.call(this, type, wrappedListener, ...args);
        }
        return originalAddEventListener.call(this, type, listener, ...args);
    };
})();

