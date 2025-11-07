package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// UDP Networking
import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.channels.DatagramChannel;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.util.Iterator;
import java.util.Set;
import java.io.IOException;

/*
 * AprilTags listens to UDP packets with April tags on a port.
 * Each UDP packet contains N AprilTag updates.
 * 
 * UDP Packet Format:
 * Header:
 *   - Number Of Tags In Packet : byte
 *   - Padding: 3 bytes
 * Each Tag: 
 *   - April Tag ID  : int (4 bytes) --- really just 0-255
 *   - Timestamp (seconds): float (4 bytes)
 *   - x (meters) : float (4 bytes)
 *   - y (meters) : float (4 bytes)
 *   - z (meters) : float ( 4 bytes)
 *   - theta (radians) : float (4 bytes)
 *   - confidence (radians) : float ( 4 bytes)
 *   - 28 bytes/tag + 4 bytes => 36 tags / packet max.
 */
public class AprilTags  extends SubsystemBase  {

    static int port = 4567;
    private DatagramChannel channel;
    private Selector selector;
    private ByteBuffer packet_buffer;


    // Define a tag.
    public class Tag {
        byte   id;
        float  timestamp;
        float  x;
        float  y;
        float  z;
        float  theta;
        float  confidence;
    }


    // Constructor:
    AprilTags() {
        try {
           channel = DatagramChannel.open();
           channel.bind(new InetSocketAddress(port)); // Listen on port 12346
           channel.configureBlocking(false); // Set to non-blocking mode

           selector = Selector.open();
           channel.register(selector, SelectionKey.OP_READ); // Register for read events


        } catch (IOException e) {
                System.out.println("AprilTags: Channel Exception");
        }
        // Preallocate our packet buffer.
        packet_buffer = ByteBuffer.allocate(1024);
    }

    //Parses the buffer from vision server into April Tags
    private Tag[] unpackData(ByteBuffer data)
    {
        //Corrects ordering of buffer
        data.order(ByteOrder.LITTLE_ENDIAN);
        //Gets how many tags are in the buffer and accounts for byte padding
        byte count = data.get();
        data.get(new byte[3]);
        Tag[] tags = new Tag[count];
        //Parses the data into a Tag object for each April Tag received
        for(int i = 0; i<count; i++)
        {
            Tag t = new Tag();
            t.id = (byte) data.getInt();
            t.timestamp = data.getFloat();
            t.x = data.getFloat();
            t.y = data.getFloat();
            t.z = data.getFloat();
            t.theta = data.getFloat();
            t.confidence = data.getFloat();
            tags[i] = t;
        }
        return tags;
    }

    @Override
    public void periodic() {
          
        try {
            int readyChannels = selector.select(); // Blocks until at least one channel is ready
            if (readyChannels == 0) return;

            Set<SelectionKey> selectedKeys = selector.selectedKeys();
            Iterator<SelectionKey> keyIterator = selectedKeys.iterator();

            while (keyIterator.hasNext()) {
                SelectionKey key = keyIterator.next();
                if (key.isReadable()) {
                    DatagramChannel readableChannel = (DatagramChannel) key.channel();
                    packet_buffer.clear();
                    readableChannel.receive(packet_buffer);
                    packet_buffer.flip();
                    
                    System.out.println("Received (NIO): " + receivedData);
                    System.out.println("Received (NIO): " + aprilTags.length);
                }
                keyIterator.remove();
            }

        } catch (IOException e) {
            System.out.println("AprilTags: Socket IO Exception");
        }
    }

}
