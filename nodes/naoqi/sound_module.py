# -*- coding: utf-8 -*-

###########################################################
# Retrieve robot audio buffer
# Syntaxe:
#    python scriptname --pip <ip> --pport <port>
# 
#    --pip <ip>: specify the ip of your robot (without specification it will use the NAO_IP defined some line below
#
# Author: Alexandre Mazel
###########################################################

NAO_IP = "10.0.252.126" # Romeo on table
#~ NAO_IP = "10.0.253.99" # Nao Alex Blue


from optparse import OptionParser
import naoqi
import numpy as np
import time
import sys


class SoundReceiverModule(naoqi.ALModule):
    """
    Use this object to get call back from the ALMemory of the naoqi world.
    Your callback needs to be a method with two parameter (variable name, value).
    """

    def __init__( self, strModuleName, strNaoIp ):
        try:
            naoqi.ALModule.__init__(self, strModuleName );
            self.BIND_PYTHON( self.getName(),"callback" );
            self.strNaoIp = strNaoIp;
            self.outfile = None;
            self.aOutfile = [None]*(4-1); # ASSUME max nbr channels = 4
        except BaseException, err:
            print( "ERR: abcdk.naoqitools.SoundReceiverModule: loading error: %s" % str(err) );

    # __init__ - end
    def __del__( self ):
        print( "INF: abcdk.SoundReceiverModule.__del__: cleaning everything" );
        self.stop();

    def start( self ):
        audio = naoqi.ALProxy( "ALAudioDevice", self.strNaoIp, 9559 );
        nNbrChannelFlag = 0; # ALL_Channels: 0,  AL::LEFTCHANNEL: 1, AL::RIGHTCHANNEL: 2; AL::FRONTCHANNEL: 3  or AL::REARCHANNEL: 4.
        nDeinterleave = 0;
        nSampleRate = 48000;
        audio.setClientPreferences( self.getName(),  nSampleRate, nNbrChannelFlag, nDeinterleave ); # setting same as default generate a bug !?!
        audio.subscribe( self.getName() );
        print( "INF: SoundReceiver: started!" );
        # self.processRemote( 4, 128, [18,0], "A"*128*4*2 ); # for local test

        # on romeo, here's the current order:
        # 0: right;  1: rear;   2: left;   3: front,  

    def stop( self ):
        print( "INF: SoundReceiver: stopping..." );
        audio = naoqi.ALProxy( "ALAudioDevice", self.strNaoIp, 9559 );
        audio.unsubscribe( self.getName() );        
        print( "INF: SoundReceiver: stopped!" );
        if( self.outfile != None ):
            self.outfile.close();


    def processRemote( self, nbOfChannels, nbrOfSamplesByChannel, aTimeStamp, buffer ):
        """
        This is THE method that receives all the sound buffers from the "ALAudioDevice" module
        """
        #~ print( "process!" );
        #~ print( "processRemote: %s, %s, %s, lendata: %s, data0: %s (0x%x), data1: %s (0x%x)" % (nbOfChannels, nbrOfSamplesByChannel, aTimeStamp, len(buffer), buffer[0],ord(buffer[0]),buffer[1],ord(buffer[1])) );
        #~ print( "raw data: " ),
        #~ for i in range( 8 ):
            #~ print( "%s (0x%x), " % (buffer[i],ord(buffer[i])) ),
        #~ print( "" );

        aSoundDataInterlaced = np.fromstring( str(buffer), dtype=np.int16 );
        #~ print( "len data: %s " % len( aSoundDataInterlaced ) );
        #~ print( "data interlaced: " ),
        #~ for i in range( 8 ):
            #~ print( "%d, " % (aSoundDataInterlaced[i]) ),
        #~ print( "" );
        aSoundData = np.reshape( aSoundDataInterlaced, (nbOfChannels, nbrOfSamplesByChannel), 'F' );
        #~ print( "len data: %s " % len( aSoundData ) );
        #~ print( "len data 0: %s " % len( aSoundData[0] ) );
        if( False ):
            # compute average
            aAvgValue = np.mean( aSoundData, axis = 1 );
            print( "avg: %s" % aAvgValue );
        if( False ):
            # compute fft
            nBlockSize = nbrOfSamplesByChannel;
            signal = aSoundData[0] * np.hanning( nBlockSize );
            aFft = ( np.fft.rfft(signal) / nBlockSize );
            print aFft;
        if( False ):
            # compute peak
            aPeakValue = np.max( aSoundData );
            if( aPeakValue > 16000 ):
                print( "Peak: %s" % aPeakValue );
        if( True ):
            bSaveAll = True;
            # save to file
            if( self.outfile == None ):
                strFilenameOut = "/out.raw";
                print( "INF: Writing sound to '%s'" % strFilenameOut );
                self.outfile = open( strFilenameOut, "wb" );
                if( bSaveAll ):
                    for nNumChannel in range( 1, nbOfChannels ):
                        strFilenameOutChan = strFilenameOut.replace(".raw", "_%d.raw"%nNumChannel);
                        self.aOutfile[nNumChannel-1] = open( strFilenameOutChan, "wb" );
                        print( "INF: Writing other channel sound to '%s'" % strFilenameOutChan );

            #~ aSoundDataInterlaced.tofile( self.outfile ); # wrote the 4 channels
            aSoundData[0].tofile( self.outfile ); # wrote only one channel
            #~ print( "aTimeStamp: %s" % aTimeStamp );
            #~ print( "data wrotten: " ),
            #~ for i in range( 8 ):
                #~ print( "%d, " % (aSoundData[0][i]) ),
            #~ print( "" );            
            #~ self.stop(); # make naoqi crashes
            if( bSaveAll ):
                for nNumChannel in range( 1, nbOfChannels ):
                    aSoundData[nNumChannel].tofile( self.aOutfile[nNumChannel-1] ); 


    # processRemote - end


    def version( self ):
        return "0.6";

# SoundReceiver - end


def main():
    """ Main entry point

    """
    parser = OptionParser()
    parser.add_option("--pip",
        help="Parent broker port. The IP address or your robot",
        dest="pip")
    parser.add_option("--pport",
        help="Parent broker port. The port NAOqi is listening to",
        dest="pport",
        type="int")
    parser.set_defaults(
        pip=NAO_IP,
        pport=9559)

    (opts, args_) = parser.parse_args()
    pip   = opts.pip
    pport = opts.pport

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    # The broker must stay alive until the program exists
    myBroker = naoqi.ALBroker("myBroker",
       "0.0.0.0",   # listen to anyone
       0,           # find a free port and use it
       pip,         # parent broker IP
       pport)       # parent broker port


    # Warning: SoundReceiver must be a global variable
    # The name given to the constructor must be the name of the
    # variable
    global SoundReceiver
    SoundReceiver = SoundReceiverModule("SoundReceiver", pip)
    SoundReceiver.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print
        print "Interrupted by user, shutting down"
        myBroker.shutdown()
        sys.exit(0)



if __name__ == "__main__":
    main()
