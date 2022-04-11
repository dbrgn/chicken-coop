// Determine server URL
const SERVER = "http://" + window.location.hostname + ":8088/janus";

// Determine stream IDs
const STREAMS = {
    picam: {
        id: 1,
        description: 'Indoor',
        wrapper: document.querySelector('#stream-picam'),
        status: document.querySelector('#stream-picam .status'),
        video: document.querySelector('#stream-picam video'),
    },
    usb: {
        id: 2,
        description: 'Outdoor',
        wrapper: document.querySelector('#stream-usb'),
        status: document.querySelector('#stream-usb .status'),
        video: document.querySelector('#stream-usb video'),
    }
}

/**
 * Set up the session. Will be called after Janus is initialized.
 */
function setupSession() {
    const janus = new Janus({
        server: SERVER,
        success: function() {
            console.info('Session setup success!');

            // Attach streaming plugin and start streams
            attachStreaming(janus, STREAMS.picam);
            attachStreaming(janus, STREAMS.usb);
        },
        error: (error) => {
            Janus.error(error);
            alert(`Error when initializing session: ${error}`);
        },
        destroyed: () => {
            window.location.reload();
        },
    });
}

/**
 * Attach to the streaming plugin and start the specified stream.
 */
function attachStreaming(janus, stream) {
    let bitrateTimer;
    let streaming;

    function stopStream() {
        streaming.send({
            message: {request: "stop"},
        });
        streaming.hangup();
        if (bitrateTimer !== undefined) {
            clearInterval(bitrateTimer);
            bitrateTimer = undefined;
        }
    }

    // Attach to streaming plugin
    const opaqueId = `chickenstream-${Janus.randomString(12)}`;
    janus.attach({
        plugin: 'janus.plugin.streaming',
        opaqueId,
        success: (pluginHandle) => {
            streaming = pluginHandle;
            console.info(`Plugin attached: ${streaming.getPlugin()} (id=${streaming.getId()})`);
            console.info(`Starting stream ${stream.description} (id=${stream.id})`);
            streaming.send({
                message: { request: "watch", id: stream.id},
            });
        },
        error: (error) => {
            Janus.error('Error attaching streaming plugin', error);
            alert(`Error attaching plugin: ${error}`);
        },
        iceState: (state) => console.debug(`ICE state changed to ${state}`),
        webrtcState: (on) => console.info(`Janus says our WebRTC PeerConnection is ${on ? 'up' : 'down'}`),
        onmessage: (msg, jsep) => {
            console.debug('Got a message', msg);

            // Handle result
            const result = msg.result;
            if (result !== undefined) {
                if (result.status !== undefined) {
                    stream.status.innerText = result.status;
                }
                if (result.status === 'stopped') {
                    stopStream();
                    return;
                }
            }

            // Handle errors
            const error = msg.error;
            if (error !== undefined) {
                console.error(`Error: ${error}`);
                stopStream();
                return;
            }

            // Handle JSEP offer
            if (jsep !== undefined) {
                const stereo = jsep.sdp.indexOf('stereo=1') !== -1;

                // Offer from the plugin, let's answer
                streaming.createAnswer(
                    {
                        jsep: jsep,
                        // We want recvonly video and no datachannels
                        media: { audioSend: false, videoSend: false, data: false },
                        customizeSdp: function(jsep) {
                            if (stereo && jsep.sdp.indexOf('stereo=1') === -1) {
                                // Make sure that our offer contains stereo too
                                jsep.sdp = jsep.sdp.replace('useinbandfec=1', 'useinbandfec=1;stereo=1');
                            }
                        },
                        success: function(jsep) {
                            var body = { request: 'start' };
                            streaming.send({
                                message: {request: 'start'},
                                jsep: jsep,
                            });
                        },
                        error: function(error) {
                            console.error('WebRTC error:', error);
                        }
                    });
            }
        },
        onremotestream: (remoteStream) => {
            console.info('Got a remote stream', remoteStream);

            // Attach media stream to video element
            Janus.attachMediaStream(stream.video, remoteStream);

            // Play
            stream.video.volume = 0;
            stream.video.play();
        },
        ondataopen: (data) => console.debug('The DataChannel is available!'),
        ondata: (data) => console.debug('We got data from the DataChannel:', data),
        oncleanup: () => {
            console.debug('Got a cleanup notification');
            if (bitrateTimer) {
                clearInterval(bitrateTimer);
            }
            bitrateTimer = undefined;
        },
    });
}

Janus.init({
    debug: true,
    dependencies: Janus.useDefaultDependencies(),
    callback: function() {
        console.info('Janus initialized!');

        // Ensure WebRTC is supported
        if (!Janus.isWebrtcSupported()) {
            alert('No WebRTC support... Cam won\'t work.');
            return;
        }

        setupSession();
    }
});
