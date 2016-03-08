//
//
//

#ifndef REGACCESS_PROTOCOL_H
#define REGACCESS_PROTOCOL_H

#include <stdint-gcc.h>

class Output {
public:
    Output() { }

    virtual void put(uint8_t byte) = 0;

    virtual ~Output() { }

private:
    Output(const Output &);

    Output &operator=(const Output &);
};

class Handler {
public:
    Handler() { }

    virtual void onReadReg(uint8_t address) = 0;

    virtual void onWriteReg(uint8_t address, uint16_t value) = 0;

    virtual ~Handler() { }

private:
    Handler(const Handler &);

    Handler &operator=(const Handler &);
};

class Protocol {
private:
    enum State {
        HEADER,
        READ_GET_ADDRESS,
        WRITE_GET_ADDRESS,
        WRITE_GET_VALUE_HI,
        WRITE_GET_VALUE_LO
    };

    Output &connection_;
    uint8_t sequence;
    uint8_t peerSequence;
    State currentState;
    Handler &handler_;
    uint8_t writeAddress;
    uint8_t writeValueHI;
    uint8_t lastHeader;

    enum {
        R = 0,
        W = 1
    };

    const uint8_t parity[256] =
            {
                    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0,
                    1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1,
                    1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1,
                    0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0,
                    1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1,
                    0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0,
                    0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1,
                    0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1,
                    1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1,
                    0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0,
                    0, 1, 0, 1, 1, 0
            };
public:
    Protocol(Output &connection, Handler &handler)
            : connection_(connection),
              sequence(0),
              peerSequence(0),
              currentState(HEADER),
              handler_(handler),
              writeAddress(0),
              writeValueHI(0) {
    }

    void writeRegister(uint8_t reg, uint16_t value) {
        uint8_t hi = value >> 8;
        uint8_t lo = value & 0xff;

        uint8_t addParity = parity[reg];
        uint8_t hiParity = parity[hi];
        uint8_t loParity = parity[lo];

        connection_.put(header(W, ++sequence, addParity, hiParity, loParity));
        connection_.put(packValue(reg));
        connection_.put(packValue(hi));
        connection_.put(packValue(lo));
    }

    void readRegister(uint8_t reg){
        connection_.put(header(R, ++sequence, parity[reg], 0, 0));
        connection_.put(packValue(reg));
    }

    void onByte(uint8_t value) {
        State nextState = currentState;
        switch (currentState) {
            case HEADER:
                if (isValidHeader(value)) {
                    lastHeader = value;
                    uint8_t expectedPeerSequence = (peerSequence + 1) & 0x3;
                    uint8_t receivedPeerSequence = getSequence(value);
                    if (expectedPeerSequence == receivedPeerSequence) {
                        if (isRead(value)) {
                            nextState = READ_GET_ADDRESS;
                        } else {
                            nextState = WRITE_GET_ADDRESS;
                        }
                    }

                    peerSequence = receivedPeerSequence;
                }
                break;

            case READ_GET_ADDRESS:
                if (parity[value] > 0) {
                    handler_.onReadReg(unpackValue(value, getAddrParity(lastHeader)));
                }
                nextState = HEADER;
                break;

            case WRITE_GET_ADDRESS:
                if (parity[value] > 0) {
                    writeAddress = unpackValue(value, getAddrParity(lastHeader));
                    nextState = WRITE_GET_VALUE_HI;
                } else {
                    nextState = HEADER;
                }
                break;

            case WRITE_GET_VALUE_HI:
                if (parity[value] > 0) {
                    writeValueHI = unpackValue(value, getHiParity(lastHeader));
                    nextState = WRITE_GET_VALUE_LO;
                } else {
                    nextState = HEADER;
                }
                break;

            case WRITE_GET_VALUE_LO:
                if (parity[value] > 0) {
                    uint8_t lo = unpackValue(value, getLoParity(lastHeader));
                    handler_.onWriteReg(writeAddress, writeValueHI << 8 | lo);
                }
                nextState = HEADER;
                break;
        }

        currentState = nextState;
    }

private:
    uint8_t packValue(uint8_t value) {
        if (parity[value] == 0) {
            if (value & 0x1) {
                value = value & (~1);
            } else {
                value |= 1;
            }
        }

        return value;
    }

    uint8_t unpackValue(uint8_t value, uint8_t expectedParity) {
        if (parity[value] == 1 && expectedParity == 0) {
            if (value & 0x1) {
                value = value & (~1);
            } else {
                value |= 1;
            }
        }

        return value;
    }

    bool isValidHeader(uint8_t header) {
        return parity[header] == 0;
    }

    bool isRead(uint8_t header) {
        return (header & 0x1) == R;
    }

    uint8_t getSequence(uint8_t header) {
        return static_cast<uint8_t>((header >> 1) & 0x3);
    }

    uint8_t getAddrParity(uint8_t header) {
        return (header >> 6) & 0x1;
    }

    uint8_t getHiParity(uint8_t header) {
        return (header >> 5) & 0x1;
    }

    uint8_t getLoParity(uint8_t header) {
        return (header >> 4) & 0x1;
    }

    uint8_t header(uint8_t RW,
                   uint8_t sequence,
                   uint8_t addrParity,
                   uint8_t hiParity,
                   uint8_t loParity) {
        uint8_t rawHeader = (RW & 0x1) |
                            ((sequence & 0x3) << 1) |
                            ((loParity & 0x1) << 4) |
                            ((hiParity & 0x1) << 5) |
                            ((addrParity & 0x1) << 6);
        //printf("Raw header 0x%x, Result header 0x%x\n", rawHeader, parity[rawHeader] << 7 | rawHeader);
        uint8_t rawHeaderParity = parity[rawHeader];
        return (rawHeaderParity << 7) | rawHeader;
    }


};


#endif //REGACCESS_PROTOCOL_H
