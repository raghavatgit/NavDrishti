/**
 * NAVDRISHTI Hardware-in-the-Loop (HIL) Bridge Server
 * Provides high-speed WebSocket streaming for real smartphones to stream
 * live MEMS sensor data directly into the NavDrishti Mission Control HUD.
 */

const http = require('http');
const fs = require('fs');
const path = require('path');
const crypto = require('crypto');

const PORT = process.env.PORT || 8088;
const STATIC_DIR = path.resolve(__dirname);

// MIME types dictionary
const MIME_TYPES = {
    '.html': 'text/html',
    '.css': 'text/css',
    '.js': 'text/javascript',
    '.json': 'application/json',
    '.png': 'image/png',
    '.jpg': 'image/jpeg',
    '.svg': 'image/svg+xml',
    '.ico': 'image/x-icon'
};

// HTTP Static & API Server
const server = http.createServer((req, res) => {
    // API endpoint for empirical datasets
    if (req.url.startsWith('/api/iovnbd-tracks')) {
        res.writeHead(200, { 
            'Content-Type': 'application/json',
            'Access-Control-Allow-Origin': '*'
        });
        res.end(JSON.stringify({
            status: "success",
            source: "https://github.com/onyekpeu/IO-VNBD",
            tracks: [
                {
                    id: "Vw13",
                    name: "M5 Motorway High-Speed Cruise (94-115 km/h)",
                    duration_s: 28.4,
                    samples: 284,
                    features: ["High-speed inertial odometry", "Zero lateral slip (NHC)", "Real CAN-bus velocity"]
                },
                {
                    id: "S1",
                    name: "Coventry Ring Road A4053 & Roundabouts",
                    distance_km: 38.16,
                    duration_min: 86.3,
                    samples: 51747,
                    features: ["9 Roundabouts", "Hard braking", "Hilly terrain", "Urban city corridors"]
                }
            ]
        }));
        return;
    }

    let filePath = path.join(STATIC_DIR, req.url === '/' ? 'index.html' : req.url.split('?')[0]);
    const ext = path.extname(filePath).toLowerCase();
    const contentType = MIME_TYPES[ext] || 'application/octet-stream';

    fs.readFile(filePath, (err, content) => {
        if (err) {
            if (err.code === 'ENOENT') {
                res.writeHead(404, { 'Content-Type': 'text/plain' });
                res.end('404 Not Found');
            } else {
                res.writeHead(500, { 'Content-Type': 'text/plain' });
                res.end(`500 Internal Server Error: ${err.code}`);
            }
        } else {
            res.writeHead(200, { 
                'Content-Type': contentType,
                'Access-Control-Allow-Origin': '*',
                'Cache-Control': 'no-cache, no-store, must-revalidate',
                'Pragma': 'no-cache',
                'Expires': '0'
            });
            res.end(content, 'utf-8');
        }
    });
});

const clients = new Set();

function formatFrame(msg) {
    const payload = Buffer.from(msg, 'utf8');
    let header;
    if (payload.length < 126) {
        header = Buffer.alloc(2);
        header[0] = 0x81; // Text frame
        header[1] = payload.length;
    } else if (payload.length <= 65535) {
        header = Buffer.alloc(4);
        header[0] = 0x81;
        header[1] = 126;
        header.writeUInt16BE(payload.length, 2);
    } else {
        header = Buffer.alloc(10);
        header[0] = 0x81;
        header[1] = 127;
        header.writeBigUInt64BE(BigInt(payload.length), 2);
    }
    return Buffer.concat([header, payload]);
}

function sendFrame(socket, msg) {
    if (socket && socket.writable) {
        socket.write(formatFrame(msg));
    }
}

function broadcast(msg, sender) {
    const frame = formatFrame(msg);
    for (const client of clients) {
        if (client !== sender && client.writable) {
            client.write(frame);
        }
    }
}

server.on('upgrade', (req, socket) => {
    if (req.headers['upgrade'] !== 'websocket') {
        socket.end('HTTP/1.1 400 Bad Request');
        return;
    }

    const key = req.headers['sec-websocket-key'];
    const hash = crypto.createHash('sha1')
        .update(key + '258EAFA5-E914-47DA-95CA-C5AB0DC85B11')
        .digest('base64');

    const headers = [
        'HTTP/1.1 101 Switching Protocols',
        'Upgrade: websocket',
        'Connection: Upgrade',
        `Sec-WebSocket-Accept: ${hash}`
    ];
    socket.write(headers.join('\r\n') + '\r\n\r\n');

    clients.add(socket);

    socket.on('data', (buffer) => {
        try {
            const isMasked = (buffer[1] & 0x80) === 0x80;
            let length = buffer[1] & 0x7f;
            let offset = 2;

            if (length === 126) {
                length = buffer.readUInt16BE(2);
                offset += 2;
            } else if (length === 127) {
                offset += 8;
            }

            let mask = null;
            if (isMasked) {
                mask = buffer.slice(offset, offset + 4);
                offset += 4;
            }

            const data = buffer.slice(offset, offset + length);
            if (isMasked && mask) {
                for (let i = 0; i < data.length; i++) {
                    data[i] ^= mask[i % 4];
                }
            }

            const message = data.toString('utf8');

            try {
                const parsed = JSON.parse(message);
                if (parsed.type === 'PING') {
                    sendFrame(socket, JSON.stringify({ type: 'PONG', t: parsed.t }));
                    return;
                }
            } catch (ignore) {}

            broadcast(message, socket);
        } catch (e) {
            // Ignore malformed frames
        }
    });

    socket.on('close', () => clients.delete(socket));
    socket.on('error', () => clients.delete(socket));
});

server.listen(PORT, () => {
    console.log(`\n======================================================`);
    console.log(`[NAVDRISHTI] Mission Control & HIL Server Active`);
    console.log(`-> Web Dashboard:               http://localhost:${PORT}`);
    console.log(`-> Mobile Live Sensor Stream:   http://localhost:${PORT}/mobile.html`);
    console.log(`======================================================\n`);
});
