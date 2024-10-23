/***
 * This function decodes the payload of an uplink/downlink message and can be inserted into your ChirpStack Server or the TTN console.
 */

function bytesToInt16(bytes, start) {
    var out = ((bytes[start]) | (bytes[start + 1] << 8));
    var sign = bytes[start + 1] & (1 << 7);
    if (sign)
        out = 0xFFFF0000 | out;
    return out;
}

function bytesToUInt16(bytes, start) {
    return ((bytes[start]) | (bytes[start + 1] << 8));
}

function bytesToInt32(bytes, start) {
    return ((bytes[start]) | (bytes[start + 1] << 8) | (bytes[start + 2] << 16) | (bytes[start + 3] << 24));
}


function decodeUplink(input) {
    var i = 0;
    var decoded = {};
    decoded.temperature = bytesToInt16(input.bytes, i) / 100;
    i += 2;
    decoded.humidity = bytesToUInt16(input.bytes, i) / 100;
    i += 2;
    decoded.voltage = bytesToInt16(input.bytes, i) / 1000.0;

    return {
        data: decoded,
        warnings: [],
        errors: []
    };
}


function decodeDownlink(input) {
    // input has the following structure:
    // {
    //   bytes: [1, 2, 3], // FRMPayload (byte array)
    //   fPort: 1
    // }
    return {
        data: {
            field: "value",
        },
        warnings: [], // optional
        errors: [], // optional (if set, the decoding failed)
    };
}
