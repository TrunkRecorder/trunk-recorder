# Object Lifecycles and States in Trunk Recorder

Calls, Recorders and the Wav file write associated with a recorder all go through a lifecycle and have different states associated with each stage of the lifecycle.

## Calls

Calls are the highest level object. They represent a frequency being reserved for use by a talkgroup. Calls in a **Conventional** systems are handled slightly differently than Calls in a **Trunked** system.

### Trunked System


#### New Calls
The Trunking Messages for each system get decoded and placed into a message queue. In *main.cc* the **monitor_messages()** pulls messages off the queue and passes them to **handle_message()** for processing. When a new **GRANT** or **UPDATE** comes in, the message is passed to **handle_call()**. 

**handle_call()** starts by going through all of the existing Calls and seeing if there is an active Call that already covers the **GRANT** or **UPDATE**. This means that the existing Call has the same:
- talkgroup
- system
- frequency
- TDMA Slot
- Phase 1 / Phase 2
... and the Call has a state or **RECORDING** or **MONITORING** (more on these in a little).

If there is an existing Call that covers all of this, then it will simply be updated.

If not, a new Call is created and an attempt to start a recording is made by calling **start_recorder()**. **start_recorder()** first checks to see if a talkgroup entry is defined in the *talkgroup.csv* file for the talkgroup number in the message. If the **recordUnknown** flag is set to false in the *config.json* and a talkgroup entry is not found, then a recording will not be started.
Encrypted transmissions are not recorded. If a talkgroup entry does exist and the talkgroup is marked as encrypted, it will be skipped. The same is true if the message's encrypted flag is set.

After these checks, **start_recorder()** will now try to assign a recorder. It does this by going through the Sources and finding the first one that covers the frequency in the message. The function will then try to get either an analog or digital recorder from the Source by calling **get_analog_recorder()** or **get_digital_recorder()**, respectively. The **get_analog_recorder()** and **get_digital_recorder()** functions in *source.cc* simply go through the vector of Recorders that were created for that Source and checking their state. The first recorder with the state of **available** is returned. For both types of recorders, their state actually comes from transmission_sink block that is at the end of the recorders gnuradio graph. We will dive into that lifecycle later. The transmission_sink block that part of the recorder, will receive some information about the call and will set its state to **idle**

Back in *main.cc*, if a recorder was return, it is associated with the call and the call's state is set to **recording**. If it wasn't possible to get a recorder, the call's state is set to **monitoring**. This serves to track that the Talkgroup will be on that frequency and that Trunk Recorder does not have to try and assign a recorder everytime it gets an **UPDATE** trunking message.

When a **GRANT** or **UPDATE** message comes in and goes through **handle_call()**, the function will see that a call has already been assigned and will call that call's **update()** function. This function simply sets the call's *last_update* variable to the current time. 

As voice samples make their way through the recorder, they eventually end up in the **transmission_sink.cc** block. When a wav_sink and its recorder are first associated with a call, its state is initially set to **idle**. When in **idle** and a voice sample comes in, the wav_sink will change its state to **recording**, open a new file and being writing samples to it. The wav_sink starts a *Transmission* which tracks each of the individual transmissions that make up a call on a digital system. The variable *d_stop_time* is also updated with the current time. This is used in analog systems to determine is there has been a break in writing to the file, signaling a new file should be started. 

In digital system, a Terminator Data Unit (TDU) is sent on the voice channel at the start and end of the call. The digital recorder processes this and passes it along to the wav_sink. When the wav_sink is in the state **recording**, signifying it has written samples to a file, and it receives a TDU it will end the current transmission. The wav_sink has an internal flag called *record_more_transmissions*. If this flag is set to *false* the wav_sink's state will be set to **stopped**, otherwise it will be set to **idle**. When a wav_sink is in state **stopped**, any voice samples that come in will be dropped and not written to a file, and new Transmission will not be created. However, when the state is **idle**, it will operate like it was just created.




## Object States

### Recorders
(we need an extra state beyond stopped. when it is stopped, a recorder could still be associated with a call and the call completer has not been called yet. it goes into available when there are no more call associated with it)
(if a Recorder misses the terminator messages, it can be stuck in RECORDING state. The main.cc idle check will clear stuck calls.)

- **available** The recorder does not have a call associated with it. There are no active transmissions.
- **idle** The recorder has an associated call. The recorder is waiting for samples to come in. There are no active transmissions. when it is in this state an receives a sample, it will start a transmission and open a file.
- **recording** The recorder has an associated call. There is an open transmission and file. When the recorder receives a Terminating Data Unit (TDU) tag, it will end the current transmission and close the file. If the *record_more_transmissions* flag has been set to *false* it will then set its state to **stopped**, if it is *true* it will set its state to **idle**
- **stopped** The recorder has an associated call. There are no active transmissions or open files. When **stop_recording()** is called, signifying that the call is complete, the state will be set back to **available**.


### Calls
- **recording** or **monitoring** - This is the initial state for a call. The call has recently received an UPDATE or GRANT message on the control channel. When the state is **recording** there is a Recorder associated with the call.
- **inactive** - An UPDATE or GRANT message has not been received for that call on the control channel within the timeout period (3 sec). The associated recorder still has the state of **recording**, meaning that it has received samples and has not yet received a TDU. The recorder's **record_more_transmissions** flag will be set to *false*. After the recorder receives a TDU or after a timeout period has elapsed (10s), the call state will transition to completed.
- **completed** - A UPDATE or GRANT message has not been received for that call on the control channel within the timeout period (3 sec). The associated recorder was in a state of **idle** or **stopped**. The recorder's **record_more_transmissions** flag gets set to *false*. In the state **completed** a call can be concluded() and deleted.
