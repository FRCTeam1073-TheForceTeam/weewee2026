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

    // Define a tag.
    public class Tag {
        int    id;
        float  timestamp;
        float  x;
        float  y;
        float  z;
        float  theta;
        float  confidence;

        public Tag(int id) {
            this.id = id;
            this.timestamp = 0.0f;
            this.x = 0.0f;
            this.y = 0.0f;
            this.z = 0.0f;
            this.theta = 0.0f;
            this.confidence = 0.0f;
        }
    }

    static int port = 4567;
    private DatagramChannel channel;
    private ByteBuffer packet_buffer;
    private int loop_counter = 0;
    private int packet_counter = 0;
    private Tag tags[] = new Tag[36]; // Tag data by ID


    // Constructor:
    public AprilTags() {
        try {
           channel = DatagramChannel.open();
           channel.configureBlocking(false); // Set to non-blocking mode
           channel.bind(new InetSocketAddress(port)); // Listen on port

        //    selector = Selector.open();
        //    channel.register(selector, SelectionKey.OP_READ); // Register for read events
        } catch (IOException e) {
                System.out.println("AprilTags: Channel Exception");
        }
        // Preallocate our packet buffer.
        packet_buffer = ByteBuffer.allocate(1024);

        // Fill out default tag buffer:
        for (int ii = 0; ii < tags.length; ++ii) {
            tags[ii] = new Tag(ii);
        }

        System.out.println("April Tags initialized...");
    }

    // Parses the buffer from vision server into internal buffer.
    private void unpackData(ByteBuffer data)
    {
        //Corrects ordering of buffer
        data.order(ByteOrder.LITTLE_ENDIAN);
        // Gets how many tags are in the buffer and accounts for byte padding
        byte count = data.get();
        data.get(new byte[3]);
        // Parses the data into a Tag object for each April Tag received
        for(int i = 0; i < count; i++)
        {
            int id = data.getInt(); // Look up the ID of the tag.
            // Only process sane/valid tag ids.
            if (id > 0 && id < tags.length) {
                tags[id].id = id;
                tags[id].timestamp = data.getFloat();
                tags[id].x = data.getFloat();
                tags[id].y = data.getFloat();
                tags[id].z = data.getFloat();
                tags[id].theta = data.getFloat();
                tags[id].confidence = data.getFloat();
            } else {
                var message = String.format("AprilTags: Received invalid tag ID %d", id);
                System.out.println(message);
            }
        }
        packet_counter++;
    }

    @Override
    public void periodic() {

        try {
            // Try to read and parse all the packets we can:
            packet_buffer.clear();
            var read_result = channel.receive(packet_buffer);
            while (read_result != null) {
                unpackData(packet_buffer);
                packet_buffer.clear();
                read_result = channel.receive(packet_buffer);
            }

        } catch (IOException e) {
            System.out.println("AprilTags: Socket IO Exception");
        }

        loop_counter += 1; // Increment loop counter.
        if (loop_counter % 400 == 0) {
            var message = String.format("April Tag Running Loops: %d, Packets: %d", 
                                        loop_counter, packet_counter);
            System.out.println(message);
        }
    }

}
