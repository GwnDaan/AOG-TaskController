-- TODO: dialog box to enter stuff wireshark can't know about (eg lightbar line distance)
-- Place in \program filew\wireshark\plugins
AOGProtocol_proto = Proto("AgOpenGPS", "AgOpenGPS Protocol")
RTCMProtocol_proto = Proto("AgOpenGPSRTCM", "AgOpenGPS RTCM Protocol")
ISOBUS_proto = Proto("ISOBUSVT", "AgISOStack")

local MajorPGNs = {
    [0x7F] = "Steer module",
    [0xFE] = "From AutoSteer",
    [0x70] = "ISOBUS"
}
local MinorPGNs = {
    [0x00] = "AOG -> ISOBUS",
    [0x80] = "ISOBUS -> AOG",
    [0xFE] = "Steer settings",
    [0xFC] = "Steer Settings"
}

local CANDebugState = {
    [0] = "Disable diagnostics",
    [1] = "Enable diagnostics",
    [2] = "Disable filters",
    [3] = "Enable filters"
}

local function to_binary(num, bits)
    local binary = ""
    for i = bits - 1, 0, -1 do
        binary = binary .. ((num & (1 << i)) ~= 0 and "1" or "0")
    end
    return binary
end

local cooked

local AOGFields = {
    AOGID = ProtoField.uint16("AOGProtocol.AOGID", "AOGID", base.HEX),
    -- AOGID2 = ProtoField.uint8("AOGProtocol.AOGID2", "AOGID2", base.HEX),
    MajorPGN = ProtoField.uint8("AOGProtocol.MajorPGN", "MajorPGN", base.HEX, MajorPGNs),
    MinorPGN = ProtoField.uint8("AOGProtocol.MinorPGN", "MinorPGN", base.HEX, MinorPGNs),

    pivotLat = ProtoField.uint16("CorrectedGPS.pivotLat", "pivotLat", base.DEC),
    pivotLon = ProtoField.uint16("CorrectedGPS.pivotLon", "pivotLon", base.DEC),
    fixHeading = ProtoField.uint16("CorrectedGPS.fixHeading", "fixHeading", base.DEC),
    pivotAlt = ProtoField.uint16("CorrectedGPS.pivotAlt", "pivotAlt", base.DEC),

    fCANDebugState = ProtoField.uint8("AOGProtocol.CANDebugState", "CANDebugState", base.DEC, CANDebugState),

    Speed = ProtoField.uint16("SteerData.Speed", "Speed", base.DEC),

    ssKp = ProtoField.uint16("SteerSettings.kp", "kp", base.DEC),
    ssHighPWM = ProtoField.uint16("SteerSettings.HighPWM", "HighPWM", base.DEC),
    ssLowPWM = ProtoField.uint16("SteerSettings.LowPWM", "LowPWM", base.DEC),
    ssMinPWM = ProtoField.uint16("SteerSettings.MinPWM", "MinPWM", base.DEC),
    ssSteerSensorCounts = ProtoField.uint16("SteerSettings.ssCounts", "Steer Sensor Counts", base.DEC),
    sswasOffset = ProtoField.uint16("SteerSettings.wasOffset", "WAS Offset", base.DEC),
    ssAckermanFix = ProtoField.uint16("SteerSettings.AckermanFix", "Ackerman Fix", base.DEC),

    fasActualSteerAngle = ProtoField.float("FAS.ActualSteerAngle", "Actual Steer Angle", base.DEC),
    fasIMUHeading = ProtoField.float("FAS.IMUHeading", "IMU Heading", base.DEC),
    fasIMURoll = ProtoField.float("FAS.IMURoll", "IMU Roll", base.DEC),
    fasSwitch = ProtoField.uint8("FAS.Switch", "Switch", base.DEC),
    fasPWMDIsplay = ProtoField.uint8("FAS.PWNDisplay", "PWM Display", base.DEC),

    vtgTrackDegrees = ProtoField.float("vtg.TrackDegrees", "Track Degrees", base.DEC),
    vtgMagneticTrack = ProtoField.float("vtg.MagneticTrack", "Magnetic Track", base.DEC),
    vtgGroundSpeedKnots = ProtoField.float("vtg.GroundSpeedKnots", "Ground Speed (knots)", base.DEC),
    vtgGoundSpeedKMH = ProtoField.float("vtg.GroundSpeedKMH", "Ground Speed (KMH)", base.DEC),

    extIMUHeading = ProtoField.uint16("extIMU.Heading", "Heading", base.DEC),
    extIMURoll = ProtoField.uint16("extIMU.Roll", "Roll (raw)", base.DEC),
    extIMUAngVel = ProtoField.uint16("extIMU.AngVel", "Angular Velocity", base.DEC),

    asActualSteeringChart = ProtoField.uint16("asActualSteeringChart", "Actual Steering Chart", base.DEC),
    asActualSteeringDegrees = ProtoField.uint16("asActualSteeringDegrees", "Actual Steering Degrees", base.DEC),
    asHeading = ProtoField.uint16("asHeading", "Heading", base.DEC),
    asRollK = ProtoField.uint16("asRollK", "Roll (raw)", base.DEC),
    asWorkSwitch = ProtoField.string("asWorkSwitch", "Work Switch", base.STRING),
    asSteerSwitch = ProtoField.string("asSteerSwitch", "Steer Switch", base.STRING),
    asPWMDisplay = ProtoField.uint16("asPWMDisplay", "PWM Display", base.DEC),

    SubNetScanConfirmation = ProtoField.bytes("SubNetScanConfirmation", "Subnet Scan Confirmation Signature (caca05)",
        base.NONE),

    PGN239_Uturn = ProtoField.uint8("PGN254_LineDistance", "Line Distance (raw)", base.DEC),
    PGN239_Speed = ProtoField.uint8("PGN239_Speed", "Speed", base.DEC),
    PGN239_HydLift = ProtoField.uint8("PGN239_HydLift", "HydLift", base.DEC),
    PGN239_Tram = ProtoField.uint8("PGN239_Tram", "Tram", base.DEC),
    PGN239_geoStop = ProtoField.uint8("PGN239_geoStop", "geoStop", base.DEC),
    PGN239_SC1to8 = ProtoField.uint8("PGN239_SC1to8", "SC1to8", base.HEX),
    PGN239_SC9to16 = ProtoField.uint8("PGN239_SC9to16", "SC9to16", base.HEX),

    PGN252_GainProportional = ProtoField.uint8("PGN239_PropGain", "Proportional Gain", base.DEC),
    PGN252_HighPWM = ProtoField.uint8("PGN239_HighPWN", "High PWM", base.DEC),
    PGN252_LowPWM = ProtoField.uint8("PGN239_LowPWM", "Low PWM", base.DEC),
    PGN252_MinPWM = ProtoField.uint8("PGN239_MinPWN", "Min PWM", base.DEC),
    PGN252_CPD = ProtoField.uint8("PGN239_CPD", "Counts Per Degree", base.DEC),
    PGN252_WasOffset = ProtoField.int16("PGN239_WasOffset", "WAS Offset", base.DEC),
    PGN252_Ackerman = ProtoField.uint8("PGN239_Ackerman", "Ackerman", base.DEC),

    PGN253_ActualSteerAngle = ProtoField.int16("PGN253_ActualSteerAngle", "Actual Steer Angle", base.DEC),

    PGN254_SpeedKM = ProtoField.string("PGN254_SpeedKM", "Speed (KM/h)", base.STRING),
    PGN254_Status = ProtoField.string("PGN254_Status", "Status", base.STRING),
    PGN254_SteerAngle = ProtoField.uint16("PGN254_SteerAngle", "Steer Angle", base.DEC),
    PGN254_LineDistance = ProtoField.uint8("PGN254_LineDistance", "Line Distance", base.DEC),
    PGN254_SC1to8 = ProtoField.uint8("PGN254_SC1to8", "SC1to8", base.HEX),
    PGN254_SC9to16 = ProtoField.uint8("PGN254_SC9to16", "SC9to16", base.HEX),

    genericIP = ProtoField.ipv4("GenericIP.IPv4", "IPv4", base.DEC),
    genericSubnet = ProtoField.ipv4("GenericIP.IPSubnet", "Subnet", base.DEC),
    genericShortIPRange = ProtoField.string("GenericIP.IPSubnet", "Subnet", base.STRING),

    freeFormMessage = ProtoField.string("Freeform.message", "Message", base.STRING)
}

FixQuality = {
    [0] = "Invalid",
    [1] = "GPS fix (SPS)",
    [2] = "DGPS fix",
    [3] = "PPS fix",
    [4] = "Real Time Kinematic",
    [5] = "Float RTK",
    [6] = "estimated (dead reckoning) (2.3 feature)",
    [7] = "Manual input mode",
    [8] = "Simulation mode"
}

local function isBitSet(byteValue, bitPosition)
    local bitMask = 2 ^ (bitPosition - 1)
    return tonumber(byteValue) % (bitMask + bitMask) >= bitMask
end

local function FormatTime(value)
    local hours = string.sub(value, 1, 2)
    local minutes = string.sub(value, 3, 4)
    local seconds = string.sub(value, 5, 6)
    local milliseconds = string.sub(value, 8)

    local formattedTime = hours .. ":" .. minutes .. ":" .. seconds .. "." .. milliseconds
    return formattedTime
end

local function AssembleIPRange(value)
    local v = tostring(value)
    local b1 = tonumber("0x" .. string.sub(v, 1, 2))
    local b2 = tonumber("0x" .. string.sub(v, 3, 4))
    local b3 = tonumber("0x" .. string.sub(v, 5, 6))
    local ret = b1 .. "." .. b2 .. "." .. b3
    return ret
end

local PandaFields = {"MessageType", "fixTime", "latitude", "LatNS", "longitude", "LonEW", "FixQuality", "numSats",
                     "HDOP", "Altitude", "AgeDGPS", "SpeedKnots", "imuHeading", "imuRoll", "imuYawRate", "StarChecksum"}
local PAOGIFields = {"MessageType", "fixTime", "latitude", "LatNS", "longitude", "LonEW", "FixQuality", "numSats",
                     "HDOP", "Altitude", "AgeDGPS", "SpeedKnots", "DualAntennaHeading", "DualAntennaRoll",
                     "DualAntennaPitch", "DualAntennaYawRate", "StarChecksum"}
local GGAFields = {"MessageType", "fixTime", "latitude", "LatNS", "longitude", "LonEW", "FixQuality", "numSats", "HDOP",
                   "AltitudeMSL", "--AltMSL", "HeightGeoid", "--AltHGE", "--Empty", "--Empty2", "StarChecksum"}
local PandaFieldsProto = {}
local PAOGIFieldsProto = {}
local GGAFieldsProto = {}

for _, fieldName in ipairs(PandaFields) do
    if fieldName == "FixQuality" then
        field = ProtoField.int8("PANDA." .. fieldName, fieldName, base.ASCII, FixQuality)
    else
        field = ProtoField.string("PANDA." .. fieldName, fieldName, base.ASCII)
    end
    table.insert(PandaFieldsProto, field)
end
for _, fieldName in ipairs(PAOGIFields) do
    if fieldName == "FixQuality" then
        field = ProtoField.int8("PAOGI." .. fieldName, fieldName, base.ASCII, FixQuality)
    else
        field = ProtoField.string("PAOGI." .. fieldName, fieldName, base.ASCII)
    end
    table.insert(PAOGIFieldsProto, field)
end
for _, fieldName in ipairs(GGAFields) do
    if fieldName == "FixQuality" then
        field = ProtoField.int8("GGA." .. fieldName, fieldName, base.ASCII, FixQuality)
    else
        field = ProtoField.string("GGA." .. fieldName, fieldName, base.ASCII)
    end
    table.insert(GGAFieldsProto, field)
end

-- Merge all field tables into a single table
local allFields = {}
for key, value in pairs(AOGFields) do
    table.insert(allFields, value)
    -- allFields[key] = value
end

for key, value in pairs(PandaFieldsProto) do
    table.insert(allFields, value)
end
for key, value in pairs(PAOGIFieldsProto) do
    table.insert(allFields, value)
end
for key, value in pairs(GGAFieldsProto) do
    table.insert(allFields, value)
end

local eInt16, encodedAngle
-- and register those fields
AOGProtocol_proto.fields = allFields

-- _              _
-- (_) ___   ___  | |__   _   _  ___
-- | |/ __| / _ \ | '_ \ | | | |/ __|
-- | |\__ \| (_) || |_) || |_| |\__ \
-- |_||___/ \___/ |_.__/  \__,_||___/

local SectionState = {
    [0] = "Disabled",
    [1] = "Enabled"
}
local ISOBUSFields = {
    ISOBUS_Section = ProtoField.uint8("ISOBUS_Section", "_", base.DEC, SectionState)
}
ISOBUS_proto.fields = ISOBUSFields

function ISOBUS_proto.dissector(buffer, pinfo, tree)
    local subtree = tree:add(ISOBUS_proto, buffer(), "ISOBUS Section info")
    local byte1 = buffer(0, 1):uint()
    local byte2 = buffer(1, 1):uint()
    local MajorPGN = buffer(2, 1):uint()
    local MinorPGN = buffer(3, 1):uint()
    subtree:add(AOGFields.AOGID, buffer(0, 2))
    subtree:add(AOGFields.MajorPGN, buffer(2, 1))
    subtree:add(AOGFields.MinorPGN, buffer(3, 1))
    pinfo.cols.info = "AgIsoStack Section Info"
    pinfo.cols.protocol = "AgIsoStack ISOBUS"
    local SRC = buffer(2, 1):uint()
    local PGN = buffer(3, 1):uint()
    if (SRC == 0x70 and PGN == 0x80) then
        local length = buffer(4, 1):uint()
        local data_start = 5
        for sectionIndex = 0, length - 1 do -- let's make the section IDs relatable, based at 1
            local sectionByte = buffer(data_start + sectionIndex, 1):uint()
            for i = 0, 7 do
                local is_bit_set = bit.band(sectionByte, bit.lshift(1, i)) ~= 0
                local stateIndex = is_bit_set and 1 or 0
                -- print(string.format("Bit %d is %s", i, is_bit_set and "set" or "not set"))
                local sectionLabel = string.format("Byte %d: Section %d:  %s", sectionIndex, i,
                    SectionState[stateIndex])
                subtree:add((" (" .. sectionLabel .. ")"))
            end
        end
    end
end

--       _
-- _ __ | |_   ___  _ __ ___
-- | '__|| __| / __|| '_ ` _ \
-- | |   | |_ | (__ | | | | | |
-- |_|    \__| \___||_| |_| |_|
function RTCMProtocol_proto.dissector(buffer, pinfo, tree)
    local subtree = tree:add(RTCMProtocol_proto, buffer(), "RTCM (NTRIP) data (not decoded)")
    pinfo.cols.info = "RTCM (NTRIP to F9P)"
    pinfo.cols.protocol = "RTCM"
end

-- __ _   ___    __ _
-- / _` | / _ \  / _` |
-- | (_| || (_) || (_| |
-- \__,_| \___/  \__, |
--              |___/

function AOGProtocol_proto.dissector(buffer, pinfo, tree)

    if buffer:len() < 4 then
        return
    end

    local subtree = tree:add(AOGProtocol_proto, buffer(), "AOG Data")

    local byte1 = buffer(0, 1):uint()
    local byte2 = buffer(1, 1):uint()
    local MajorPGN = buffer(2, 1):uint()
    local MinorPGN = buffer(3, 1):uint()
    subtree:add(AOGFields.AOGID, buffer(0, 2))
    subtree:add(AOGFields.MajorPGN, buffer(2, 1))
    subtree:add(AOGFields.MinorPGN, buffer(3, 1))
    if (byte1 == 0x24 and byte2 == 0x50 and MinorPGN == 0x4e) then -- PANDA
        local PandaString = buffer(0):string()
        local values = {}
        for value in PandaString:gmatch("[^,]+") do
            table.insert(values, value)
        end
        values[2] = FormatTime(values[2])
        -- rewrite latitude
        local mul
        local degrees = tonumber(string.sub(values[3], 1, 2))
        local minutes = tonumber(string.sub(values[3], 3)) or 0
        if values[4] == "S" then
            values[3] = degrees + (minutes / 60) * -1
        else
            values[3] = degrees + (minutes / 60)
        end
        -- rewrite longitude
        local degrees = tonumber(string.sub(values[5], 1, 2))
        local minutes = tonumber(string.sub(values[5], 3)) or 0
        if values[6] == "W" then
            values[5] = degrees + (minutes / 60) * -1
        else
            values[5] = degrees + (minutes / 60)
        end
        for i, value in ipairs(values) do
            subtree:add(PandaFieldsProto[i], values[i])
        end
        pinfo.cols.info = "AOG PANDA Location"
    elseif (byte1 == 0x24 and byte2 == 0x50 and MinorPGN == 0x4f) then -- PAOGI
        local PAOGIString = buffer(0):string()
        local values = {}
        for value in PAOGIString:gmatch("[^,]+") do
            table.insert(values, value)
        end
        values[2] = FormatTime(values[2])
        -- rewrite latitude
        local mul
        local degrees = tonumber(string.sub(values[3], 1, 2))
        local minutes = tonumber(string.sub(values[3], 3)) or 0
        if values[4] == "S" then
            values[3] = degrees + (minutes / 60) * -1
        else
            values[3] = degrees + (minutes / 60)
        end
        -- rewrite longitude
        local degrees = tonumber(string.sub(values[5], 1, 2))
        local minutes = tonumber(string.sub(values[5], 3)) or 0
        if values[6] == "W" then
            values[5] = degrees + (minutes / 60) * -1
        else
            values[5] = degrees + (minutes / 60)
        end
        for i, value in ipairs(values) do
            subtree:add(PAOGIFieldsProto[i], values[i])
        end
        pinfo.cols.info = "AOG PAOGI Location"
    elseif byte1 == 0x24 and byte2 == 0x47 and MinorPGN == 0x56 then -- GPVTG
        local VTG = buffer(0):string()
        local values = {}
        for value in VTG:gmatch("[^,]+") do
            table.insert(values, value)
        end
        subtree:add(AOGFields.vtgTrackDegrees, tonumber(values[2]))
        subtree:add(AOGFields.vtgMagneticTrack, tonumber(values[4]))
        subtree:add(AOGFields.vtgGroundSpeedKnots, tonumber(values[6]))
        subtree:add(AOGFields.vtgGoundSpeedKMH, tonumber(values[8]))
        pinfo.cols.info = "VTG Speed/Heading response"
    elseif byte1 == 0x24 and byte2 == 0x47 and MinorPGN == 0x47 then -- GPPGA
        local GGAString = buffer(0):string()
        local values = {}
        for value in GGAString:gmatch("[^,]+") do
            table.insert(values, value)
        end
        values[2] = FormatTime(values[2])
        -- rewrite latitude
        local mul
        local degrees = tonumber(string.sub(values[3], 1, 2))
        local minutes = tonumber(string.sub(values[3], 3)) or 0
        if values[4] == "S" then
            values[3] = degrees + (minutes / 60) * -1
        else
            values[3] = degrees + (minutes / 60)
        end
        -- rewrite longitude
        local degrees = tonumber(string.sub(values[5], 1, 2))
        local minutes = tonumber(string.sub(values[5], 3)) or 0
        if values[6] == "W" then
            values[5] = degrees + (minutes / 60) * -1
        else
            values[5] = degrees + (minutes / 60)
        end
        for i, value in ipairs(values) do
            if (string.sub(GGAFields[i], 1, 2) ~= "--") then
                subtree:add(GGAFieldsProto[i], values[i])
            end
        end
        pinfo.cols.info = "AOG GGA Location response"

    elseif byte1 == 0x80 and byte2 == 0x81 then -- we're into PGNs from AOG now
        if MajorPGN == 0x70 then -- from ISOBUS
            pinfo.cols.info = "ISOBUS data"
            if MinorPGN == 0x00 then -- 0
                pinfo.cols.info = "AOG -> ISOBUS data"
            end
            if MinorPGN == 0x80 then -- 128
                pinfo.cols.info = "*************** ISOBUS -> AOG data"
            end
        end
        if MajorPGN == 0x7f then -- steer module
            
            if MinorPGN == 0x64 then
                pinfo.cols.info = "GPSOut"
            elseif MinorPGN == 0xaa then -- 170
                pinfo.cols.info = "CANBUS manufacturer change to brand id:" .. buffer(5, 1)
            end
            if MinorPGN == 0xab then -- 171
                pinfo.cols.info = "CANBUS query manufacturer response: " .. buffer(5, 1)
            end
            if MinorPGN == 0xac then -- 172
                pinfo.cols.info = "CANBUS Set logging state"
                subtree:add(AOGFields.fCANDebugState, buffer(5, 1))
            end
            if MinorPGN == 0xc7 then -- 199
                pinfo.cols.info = "Hello from AgIO!"
            end
            if MinorPGN == 0xc8 then -- 200
                pinfo.cols.info = "Hello from AgOpenGPS to all!"
            end
            if MinorPGN == 0xc9 then -- 201
                subtree:add(AOGFields.genericIP, buffer(7, 4))
                pinfo.cols.info = "Subnet change"
            end
            if MinorPGN == 0xca then -- 202
                subtree:add(AOGFields.SubNetScanConfirmation, buffer(5, 3))
                pinfo.cols.info = "Subnet scan request"
            end
            if MinorPGN == 0xcb then -- 203 THIS DOESN'T LOOK LIKE REQUIRED HERE, IT'S A 7E REPLY IN CODE
                pinfo.cols.info = "Subnet scan reply"
            end
            if MinorPGN == 0xd0 then -- 208
                pinfo.cols.info = "Corrected GPS data"
                encodedAngle = buffer(5, 4):le_uint()
                subtree:add(AOGFields.pivotLat, (encodedAngle * 0.0000001) - 210)
                encodedAngle = buffer(9, 4):le_uint()
                subtree:add(AOGFields.pivotLon, (encodedAngle * 0.0000001) - 210)
                eInt16 = buffer(13, 2):le_uint()
                subtree:add(AOGFields.fixHeading, (eInt16 / 128))
                eInt16 = buffer(15, 2):le_uint()
                subtree:add(AOGFields.pivotAlt, (eInt16 * 0.01))
            end
            if MinorPGN == 0xd3 then -- e211
                cooked = buffer(5, 2):le_uint() * 0.1
                subtree:add(AOGFields.extIMUHeading, cooked)
                cooked = buffer(7, 2):le_uint() * 0.1
                subtree:add(AOGFields.extIMURoll, cooked)
                cooked = buffer(9, 2):le_uint() / -2
                subtree:add(AOGFields.extIMUAngVel, cooked)
                pinfo.cols.info = "External IMU"
            end
            if MinorPGN == 0xd4 then -- 212
                pinfo.cols.info = "External IMU disconnect"
            end

            if MinorPGN == 0xe5 then -- 229
                pinfo.cols.info = "64 Section on/off"
            end

            if MinorPGN == 0xe7 then -- 231
                pinfo.cols.info = "Tool Settings"
            end
            if MinorPGN == 0xea then -- 234
                pinfo.cols.info = "Matt Switch Module"
            end
            if MinorPGN == 0xeb then -- 235
                pinfo.cols.info = "Section Dimensions"
            end
            if MinorPGN == 0xec then -- 236
                pinfo.cols.info = "Pin Config"
            end
            if MinorPGN == 0xed then -- 237
                pinfo.cols.info = "From Machine Module"
            end
            if MinorPGN == 0xee then -- 238
                pinfo.cols.info = "Machine config user data"
            end
            if MinorPGN == 0xef then -- 239
                subtree:add(AOGFields.PGN239_Uturn, buffer(5, 1))
                subtree:add(AOGFields.PGN239_Speed, buffer(6, 1))
                subtree:add(AOGFields.PGN239_HydLift, buffer(7, 1))
                subtree:add(AOGFields.PGN239_Tram, buffer(8, 1))
                subtree:add(AOGFields.PGN239_geoStop, buffer(9, 1))
                -- 10 is commented out in AOG for some reason
                subtree:add(AOGFields.PGN239_SC1to8, buffer(11, 1)):append_text("  (Sections: " ..
                                                                                    to_binary(buffer(11, 1):uint(), 8) ..
                                                                                    ")")
                subtree:add(AOGFields.PGN239_SC9to16, buffer(12, 1)):append_text("  (Sections: " ..
                                                                                     to_binary(buffer(12, 1):uint(), 8) ..
                                                                                     ")")
                pinfo.cols.info = "Machine Data"
            end
            if MinorPGN == 0xfa then -- 250
                pinfo.cols.info = "Steer config (not used in AOG much)"
            end
            if MinorPGN == 0xfb then -- 251
                pinfo.cols.info = "Autosteer steer config"
            end
            if MinorPGN == 0xfc then -- 252
                subtree:add(AOGFields.PGN252_GainProportional, buffer(5, 1))
                subtree:add(AOGFields.PGN252_HighPWM, buffer(6, 1))
                subtree:add(AOGFields.PGN252_LowPWM, buffer(7, 1))
                subtree:add(AOGFields.PGN252_MinPWM, buffer(8, 1))
                subtree:add(AOGFields.PGN252_CPD, buffer(9, 1))
                subtree:add(AOGFields.PGN252_WasOffset, buffer(10, 2))
                subtree:add(AOGFields.PGN252_Ackerman, buffer(12, 1))
                pinfo.cols.info = "Steer settings (from AgIO)"
            end
            if MinorPGN == 0xfd then -- 253
                pinfo.cols.info = "Steer settings (returned from Teensy)"
                cooked = buffer(5, 2):le_uint() * 0.1
                subtree:add(AOGFields.asActualSteeringChart, cooked)
                cooked = buffer(7, 2):le_uint() -- * 0.1
                subtree:add(AOGFields.asHeading, cooked)
                cooked = buffer(9, 2):le_uint() -- * 0.1
                subtree:add(AOGFields.asRollK, cooked)
                -- print("buffer ",buffer(11,1))
                if isBitSet(buffer(11, 1):int(), 1) then
                    subtree:add(AOGFields.asSteerSwitch, "Set")
                else
                    subtree:add(AOGFields.asSteerSwitch, "Unset")
                end
                -- print("!")
                if isBitSet(buffer(11, 1):int(), 2) then
                    subtree:add(AOGFields.asWorkSwitch, "Set")
                else
                    subtree:add(AOGFields.asWorkSwitch, "Unset")
                end
                subtree:add(AOGFields.asRollK, cooked)

            end
            if MinorPGN == 0xfe then -- 254
                local speed = buffer(5, 2):le_uint() / 10
                subtree:add(AOGFields.PGN254_SpeedKM, buffer(5, 2), string.format("%.1f km/h", speed))
                if buffer(7, 1):int() == 1 then
                    subtree:add(AOGFields.PGN254_Status, "On")
                else
                    subtree:add(AOGFields.PGN254_Status, "Off")
                end
                subtree:add(AOGFields.PGN254_SteerAngle, buffer(8, 2):le_uint())
                subtree:add(AOGFields.PGN254_LineDistance, buffer(10, 1))
                subtree:add(AOGFields.PGN254_SC1to8, buffer(11, 1)):append_text("  (Sections: " ..
                                                                                    to_binary(buffer(11, 1):uint(), 8) ..
                                                                                    ")")
                subtree:add(AOGFields.PGN254_SC9to16, buffer(12, 1)):append_text("  (Sections: " ..
                                                                                     to_binary(buffer(12, 1):uint(), 8) ..
                                                                                     ")")
                pinfo.cols.info = "Steer data (from AgIO)"
            end
        end

        if MajorPGN == 0x78 then -- From GPS module 120
        end

        if MajorPGN == 0x79 then
            if MinorPGN == 0x79 then
                pinfo.cols.info = "Hello from IMU!"
            end
        end
        -- woah, watch out here, as it's back-checking data[2] rather than data[3], it's a TRAP!
        -- some weird shit going on here, see #403 ReceiveFromLoopBack

        if MajorPGN == 0x7b then -- Machine Module 123 
            subtree:add(AOGFields.genericIP, buffer(5, 4))
            subtree:add(AOGFields.genericSubnet, buffer(9, 3))
            if MinorPGN == 0x7b then -- 123
                pinfo.cols.info = "Hello from Machine board!"
            end
            pinfo.cols.info = "Machine Info"
        end

        if MajorPGN == 0x7C then -- 124 From GPS
        end

        if MajorPGN == 0x7D then -- 125 From IMU
        end

        if MajorPGN == 0x7e then -- 126
            if MinorPGN == 0xcb then
                subtree:add(AOGFields.genericIP, buffer(5, 4))
                subtree:add(AOGFields.genericShortIPRange, AssembleIPRange(buffer(5, 3)))
                pinfo.cols.info = "Subnet scan reply"
            end
            if MinorPGN == 0x7a then
                pinfo.cols.info = "7B / 7E mystery!"
            end
            if MinorPGN == 0xFD then -- 253
                subtree:add(AOGFields.fasActualSteerAngle, buffer(5, 2):le_uint() * 0.01)
                subtree:add(AOGFields.fasIMUHeading, buffer(7, 2):le_uint())
                subtree:add(AOGFields.fasIMURoll, buffer(9, 2):le_uint())
                subtree:add(AOGFields.fasPWMDIsplay, buffer(12, 1))
                pinfo.cols.info = "Steer Data"
            end
            if MinorPGN == 0x7E then -- 126
                -- hol' up, wait a minute, somethin ain't right!
                -- Shouldn't this be a Major 7F response????
                -- not enough fields coming back??? eh??? and IP isn't right either!
                subtree:add(AOGFields.genericIP, buffer(5, 4))
                -- print("7e says " .. buffer(5,4))
                -- subtree:add(AOGFields.genericSubnet, buffer(9))
                pinfo.cols.info2 = "Subnet Scan Reply"
            end
        end

        if MajorPGN == 0x7e then
            if MinorPGN == 0x7e then
                pinfo.cols.info = "Steer Reply"
            end
        end
        if MajorPGN == 0x41 then -- 65
            subtree:add(SteerData.Speed, buffer(4, 2))
        end
    elseif byte1 == 0x80 and byte2 == 0x99 then -- freeform message from Teensy!
        pinfo.cols.info = "AOG: " .. buffer(2, buffer:len() - 2):string()
        subtree:add(AOGFields.freeFormMessage, buffer(2, buffer:len() - 2))
    end
    -- Set the protocol description in the packet details pane
    pinfo.cols.protocol = AOGProtocol_proto.name

end

-- Register the AOGProtocol dissector
local udp_port = DissectorTable.get("udp.port")
udp_port:add(9999, AOGProtocol_proto)
udp_port:add(2233, RTCMProtocol_proto)
--udp_port:add(8888, ISOBUS_proto)

