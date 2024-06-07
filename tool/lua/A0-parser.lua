-- Copyright (c) 2014-2022, Asensing Group
--
-- Asensing LiDAR protocol plugin for Wireshark
--
-- Change Logs:
-- Date           Author       Notes
-- 2022-09-13     luhuadong    the first version
-- 2022-09-22     luhuadong    support little-endian
-- 2023-03-06     luhuadong    support 5 laser modules
-- 2023-06-19     luhuadong    add lidar info field


-- Declare our protocol
lidar_proto = Proto("Asensing","Asensing LiDAR Protocol")

-- Header fields
sob = ProtoField.none ("asensing.sob", "Sob", base.HEX)
frame_id = ProtoField.uint32 ("asensing.frameid", "FrameID", base.DEC)
seq_num = ProtoField.uint16 ("asensing.seqnum", "SeqNum", base.DEC)
pkg_len = ProtoField.uint16 ("asensing.pkglen", "PkgLen", base.DEC)
lidar_type = ProtoField.uint8 ("asensing.lidar_type", "LidarType", base.DEC)
lidar_info = ProtoField.uint8 ("asensing.lidar_info", "LidarInfo", base.DEC)
version_major = ProtoField.uint8 ("asensing.version_major", "VersionMajor", base.DEC)
version_minor = ProtoField.uint8 ("asensing.version_minor", "VersionMinor", base.DEC)
utc_time0 = ProtoField.uint8 ("asensing.utc_time0", "UTCTime0", base.DEC)
utc_time1 = ProtoField.uint8 ("asensing.utc_time1", "UTCTime1", base.DEC)
utc_time2 = ProtoField.uint8 ("asensing.utc_time2", "UTCTime2", base.DEC)
utc_time3 = ProtoField.uint8 ("asensing.utc_time3", "UTCTime3", base.DEC)
utc_time4 = ProtoField.uint8 ("asensing.utc_time4", "UTCTime4", base.DEC)
utc_time5 = ProtoField.uint8 ("asensing.utc_time5", "UTCTime5", base.DEC)
time_stamp = ProtoField.uint32 ("asensing.time_stamp", "Timestamp", base.DEC)
measure_mode = ProtoField.uint8 ("asensing.measure_mode", "MeasureMode", base.DEC)
laser_num = ProtoField.uint8 ("asensing.laser_num", "LaserNum", base.DEC)
block_num = ProtoField.uint8 ("asensing.block_num", "BlockNum", base.DEC)
echo_count = ProtoField.uint8 ("asensing.echo_count", "EchoCount", base.DEC)
time_sync_mode = ProtoField.uint8 ("asensing.time_sync_mode", "TimeSyncMode", base.DEC)
time_sync_stat = ProtoField.uint8 ("asensing.time_sync_stat", "TimeSyncStat", base.DEC)
mems_temp = ProtoField.uint8 ("asensing.mems_temp", "MemsTemp", base.DEC)
slot_num = ProtoField.uint8 ("asensing.slot_num", "SlotNum", base.DEC)
point_num = ProtoField.uint32 ("asensing.point_num", "PointNum", base.DEC)
reserved1 = ProtoField.uint16 ("asensing.reserved1", "Reserved1", base.DEC)

-- Block fields
distance = ProtoField.uint16 ("asensing.block.unit.distance", "Distance", base.DEC)
azimuth = ProtoField.uint16 ("asensing.block.unit.azimuth", "Azimuth", base.DEC)
elevation = ProtoField.uint16 ("asensing.block.unit.elevation", "Elevation", base.DEC)
intensity = ProtoField.uint8 ("asensing.block.unit.intensity", "Intensity", base.DEC)
unit_reserved = ProtoField.uint16 ("asensing.block.unit.reserved", "Reserved", base.DEC)


lidar_proto.fields = {
    -- Header
    sob, frame_id, seq_num, pkg_len, lidar_type, lidar_info, version_major, version_minor, 
    utc_time0, utc_time1, utc_time2, utc_time3, utc_time4, utc_time5, time_stamp, 
    measure_mode, laser_num, block_num, echo_count, time_sync_mode, time_sync_stat, 
    mems_temp, slot_num, point_num, reserved1,
    -- Block
    distance, azimuth, elevation, intensity, unit_reserved
}

-- Create a function to dissect it
function lidar_proto.dissector(buffer, pinfo, tree)
    length = buffer:len()
    if length == 0 then return end
    
    pinfo.cols.protocol = lidar_proto.name;
    local subtree = tree:add(lidar_proto, buffer(),"Asensing LiDAR packet data")

    local curr = 0

    local headerSize = 40

    local blockPerPacket = 12
    
    local nbLaser = 10
    local laserSize = 9
    local blockSize =  4 + nbLaser * laserSize
    
    local tailSize = 4

    -- Packet Header --
    local header_subtree = subtree:add_le(buffer(curr, headerSize), "Header")

    header_subtree:add_le(sob, buffer(curr, 4))
    curr = curr + 4

    header_subtree:add_le(frame_id, buffer(curr, 4))
    curr = curr + 4

    header_subtree:add_le(seq_num, buffer(curr, 2))
    curr = curr + 2

    header_subtree:add_le(pkg_len, buffer(curr, 2))
    curr = curr + 2

    header_subtree:add_le(lidar_type, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add_le(lidar_info, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(version_major, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(version_minor, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(utc_time0, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(utc_time1, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(utc_time2, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(utc_time3, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(utc_time4, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(utc_time5, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add_le(time_stamp, buffer(curr, 4))
    curr = curr + 4

    header_subtree:add(measure_mode, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(laser_num, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(block_num, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(echo_count, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(time_sync_mode, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(time_sync_stat, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(mems_temp, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add(slot_num, buffer(curr, 1))
    curr = curr + 1

    header_subtree:add_le(point_num, buffer(curr, 4))
    curr = curr + 4

    header_subtree:add(reserved1, buffer(curr, 2))
    curr = curr + 2

    ---- bock Return ----
    local size = blockPerPacket * blockSize
    local blockreturns = subtree:add(buffer(curr, size), "Blocks")

    for i=0, blockPerPacket-1
    do
        local block_subtree = blockreturns:add(buffer(curr,blockSize), "Block Return : " ..i)

        local channelNum = buffer(curr, 1):uint()
        block_subtree:add(buffer(curr, 1), "channelNum : " .. channelNum)
        curr = curr + 1

        local timeOffSet = buffer(curr, 1):uint()
        block_subtree:add(buffer(curr, 1), "timeOffSet : " .. timeOffSet)
        curr = curr + 1

        local returnSn = buffer(curr, 1):uint()
        block_subtree:add(buffer(curr, 1), "returnSn : " .. returnSn)
        curr = curr + 1

        local reserved = buffer(curr, 1):uint()
        block_subtree:add(buffer(curr, 1), "reserved : " .. reserved)
        curr = curr + 1
        
        local all_laser_subtree = block_subtree:add(buffer(curr, laserSize * nbLaser), "All Lasers Return")

        for j=0, nbLaser-1
        do
            local laser_subtree = all_laser_subtree:add(buffer(curr, laserSize), "Laser Return : " ..j)

            laser_subtree:add_le(distance, buffer(curr, 2))
            curr = curr + 2

            laser_subtree:add_le(azimuth, buffer(curr, 2))
            curr = curr + 2

            laser_subtree:add_le(elevation, buffer(curr, 2))
            curr = curr + 2

            laser_subtree:add_le(intensity, buffer(curr, 1))
            curr = curr + 1

            laser_subtree:add_le(unit_reserved, buffer(curr, 2))
            curr = curr + 2
        end

    end


    -- Tail --
    local tail_subtree = subtree:add(buffer(curr, tailSize), "Tail")
    
    --[[
    local CRC = buffer(curr, 4):uint()
    tail_subtree:add(buffer(curr,4),"CRC : " .. CRC)
    curr = curr + 4

    for n=0, 16
    do
        local functionSafety_subtree = tail_subtree:add(buffer(curr,1),"functionSafety subtree : " ..n)

        local functionSafety = buffer(curr,1):uint()
        functionSafety_subtree:add(buffer(curr,1),"functionSafety  : " .. functionSafety)
        curr = curr + 1
    end
    --]]

    local TailFlag = buffer(curr, 4):uint()
    tail_subtree:add(buffer(curr, 4), "TailFlag : " .. TailFlag)
    curr = curr + 4

end


-- load the udp.port table
udp_table = DissectorTable.get("udp.port")
-- register our protocol to handle udp port 51180
udp_table:add(51180, lidar_proto)
