// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.devices.cosmetic.music;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BreakerLib.util.logging.advantagekit.BreakerLog;

/** Play audio via the speakers on CTRE Falcon 500 motors. */
public class BreakerFalconOrchestra extends SubsystemBase {

    private Orchestra orchestra;
    private String curSong;
    private boolean loopSong, playingMusic;

    private String[] playlist = new String[]{};
    private int index;
    private boolean loopAtEnd = true, autoplay = true;

    /** Creates an empty Falcon orchestra. */
    public BreakerFalconOrchestra() {
        orchestra = new Orchestra();
    }

    /**
     * Creates an Orchestra with the provided Falcons.
     * 
     * @param motors Falcon motors to include in the orchestra.
     */
    public BreakerFalconOrchestra(TalonFX... motors) {
        orchestra = new Orchestra();
        addOrchestraMotors(motors);
    }

    /**
     * @return Song playtime in milliseconds. If the song was stopped/never played,
     *         this value will be 0.
     */
    public int getPlaytimeMS() {
        return orchestra.getCurrentTime();
    }

    /** @return If music is currently playing. */
    public boolean isPlaying() {
        return orchestra.isPlaying();
    }

    /** @return If music is currently paused in the middle of playback. */
    public boolean isPaused() {
        return !isPlaying() && getPlaytimeMS() != 0;
    }

    /** @return If music playback was stopped. */
    public boolean isStopped() {
        return !isPlaying() && getPlaytimeMS() == 0;
    }

    /** @return If music is stopped due to a song ending. */
    public boolean awaitingSong() {
        return isStopped() && playingMusic;
    }

    /**
     * Adds motors to the already-created Orchestra.
     * 
     * @param motors Falcon motors to add.
     */
    public void addOrchestraMotors(TalonFX... motors) {
        for (TalonFX motor : motors) {
            orchestra.addInstrument(motor);
        }
    }

    /** Removes all motors from the orchestra. */
    public void clearOrchestraMotors() {
        orchestra.clearInstruments();
    }

    /**
     * Plays currently loaded music file. Will fail if a music file is not loaded.
     */
    public void play() {
        playingMusic = true;
        orchestra.play();
    }

    /**
     * Loads a song through the given filepath.
     * 
     * @param songPath Path to the Chirp file on the RoboRIO. Files must be within
     *                 the deploy directory of the robot project and have the
     *                 extension .chrp.
     */
    public void loadSong(String songPath) {
        curSong = songPath;
        orchestra.loadMusic(songPath);
    }

    /** Pauses song playback allowing for music to be resumed later. */
    public void pause() {
        playingMusic = false;
        orchestra.pause();
    }

    /** Stops playing music and moves timestamp back to the beginning. */
    public void stop() {
        playingMusic = false;
        orchestra.stop();
    }

    /**
     * Set if songs will loop or not.
     * 
     * @param loop Whether to loop music.
     */
    public void setLooping(boolean loop) {
        loopSong = loop;
    }

    /**
     * Plays the given song.
     * 
     * @param songPath Path to the Chirp file on the RoboRIO. Files must be within
     *                 the deploy directory of the robot project and have the
     *                 extension .chrp.
     */
    public void playSong(String songPath) {
        loadSong(songPath);
        play();
    }

    /**
     * Loops the given song.
     * 
     * @param songPath Path to the Chirp file on the RoboRIO. Files must be within
     *                 the deploy directory of the robot project and have the
     *                 extension .chrp.
     */
    public void loopSong(String songPath) {
        setLooping(true);
        playSong(songPath);
    }

    // PLAYLIST

    /**
     * Loads a playlist of songs to access and play.
     * 
     * @param playlist Paths to Chirp files.
     */
    public void loadPlaylist(String... playlist) {
        this.playlist = playlist;
    }

    /**
     * Plays the desired song in the playlist. Will fail if the song index is
     * invalid.
     * 
     * @param index Playlist index to access.
     */
    public void startPlaylistSong(int index) {
        try {
            this.index = index;
            curSong = playlist[index];
            playSong(curSong);
        } catch (IndexOutOfBoundsException e) {
            startPlaylistSong(makeLoopIndex(index));
        }
    }

    /** Plays first song in the playlist. */
    public void startPlaylist() {
        startPlaylistSong(0);
    }

    /** Plays the next song in the playlist. */
    public void playNextSong() {
        startPlaylistSong(index + 1);
    }

    /** Plays the previous song in the playlist. */
    public void playPreviousSong() {
        startPlaylistSong(index - 1);
    }

    /**
     * Sets whether the playlist will automatically proceed to the next song.
     * 
     * @param value True for autoplay, false for no autoplay.
     */
    public void setAutoplay(boolean value) {
        autoplay = value;
    }

    /**
     * Sets whether the playlist will loop at the end or simply stop.
     * 
     * @param value True for looping at end, false for no looping at end. Playlists
     *              can still be looped manually.
     */
    public void setLoopAtEnd(boolean value) {

    }

    /**
     * Creates an index within playlist bounds.
     * 
     * @param index Index to operate on.
     * @return Bounds-compliant index.
     */
    private int makeLoopIndex(int index) {
        int len = playlist.length;
        return index < 0 ? len - 1 : index % len;
    }

    /**
     * @param index Index to check.
     * @return If index is outside of playlist bounds.
     */
    private boolean indexOutOfBounds(int index) {
        return index < 0 || index > playlist.length;
    }

    @Override
    public void periodic() {
        // Loops song if song should loop.
        if (awaitingSong() && loopSong) {
            playSong(curSong);
        }
        // Queues next song if next song should be queued.
        else if (autoplay) {
            if (!loopAtEnd && indexOutOfBounds(index + 1)) {
                stop();
            } else {
                playNextSong();
            }
        }
    }
}
