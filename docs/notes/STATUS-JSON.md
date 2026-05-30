Status json messages
=======================


The json message are sent over a websocket to the server specified by **statusServer** config entry.

The following message types are sent 
* **config**
  * Contains the config information
  * Sent when the socket is first connected
* **rates**
  * Contains message decode rates for all systems, non trucked systems will have a zero value
  * Sent every 3 seconds
* **systems**
  * Contains an array of systems 
  * Sent when the socket is first connected
* **system**
  * Contains a single system
  * Sent when a system changes, this happens when the sysid, wacn, or nac is first known
* **calls_active**
  * Contains an array of all calls that are currently active
  * Sent when the socket is first connected, a call is started, or call is completed
* **call_start**
  * Contains a single call
  * Sent when a call is started
* **call_end**
  * Contains a single call
  * Sent when a call is completed
* **recorders**
  * Contains an array of all recorders
  * Sent when the socket is first connected
* **recorder**
  * Contains a single recorder
  * Sent when a record has changed


## config
 ```
 *not documented yet*
 ```

## rates
```json
{
    "rates": [
        {
            "id": "0",
            "decoderate": "39.333332"
        },
        {
            "id": "1",
            "decoderate": "0"
        }
    ],
    "type": "rates",
    "instanceId": "",
    "instanceKey": ""
}
```

## systems
```json
{
    "systems": [
        {
            "id": "0",
            "name": "SYS 1",
            "type": "p25",
            "sysid": "123",
            "wacn": "456",
            "nac": "789012"
        },
        {
            "id": "1",
            "name": "SYS 2",
            "type": "conventionalP25",
            "sysid": "0",
            "wacn": "0",
            "nac": "0"
        }
    ],
    "type": "systems",
    "instanceId": "",
    "instanceKey": ""
}
```

## system
```json
{
    "system": {
            "id": "0",
            "name": "SYS 1",
            "type": "p25",
            "sysid": "123",
            "wacn": "456",
            "nac": "789012"
        },
    "type": "system",
    "instanceId": "",
    "instanceKey": ""
}

```

## calls_active
```json
{
    "calls": [
        {
            "id": "1_1_1515574626",
            "freq": "410000000",
            "sysNum": "1",
            "shortName": "SYS 2",
            "talkgroup": "1",
            "talkgrouptag": "TG 05",
            "elasped": "46",
            "length": "25.632000000000001",
            "state": "1",
            "phase2": "false",
            "conventional": "true",
            "encrypted": "false",
            "emergency": "false",
            "startTime": "1515574626",
            "stopTime": "1515574626",
            "freqList": "",
            "recNum": "8",
            "srcNum": "0",
            "recState": "3",
            "analog": "false",
            "filename": "\/home\/\/*****\/\/captures\/SYS 2\/2018\/1\/10\/1-1515574626_4.1e+08.wav",
            "statusfilename": "\/home\/\/*****\/\/captures\/SYS 2\/2018\/1\/10\/1-1515574626_4.1e+08.json"
        },
        {
            "id": "0_1001_1515574637",
            "freq": "419000000",
            "sysNum": "0",
            "shortName": "SYS 1",
            "talkgroup": "1001",
            "talkgrouptag": "TG 77",
            "elasped": "35",
            "length": "25.091999999999999",
            "state": "1",
            "phase2": "false",
            "conventional": "false",
            "encrypted": "false",
            "emergency": "false",
            "startTime": "1515574637",
            "stopTime": "1515574637",
            "freqList": [
                {
                    "freq": "419000000",
                    "time": "1515574637"
                }
            ],
            "recNum": "0",
            "srcNum": "0",
            "recState": "3",
            "analog": "false",
            "filename": "\/home\/*****\/captures\/SYS 1\/2018\/1\/10\/1001-1515574637_4.19e+08.wav",
            "statusfilename": "\/home\/****\/captures\/SYS 1\/2018\/1\/10\/1001-1515574637_4.19e+08.json"
        }
    ],
    "type": "calls_active",
    "instanceId": "",
    "instanceKey": ""
}
```

## call_start
```json
{
    "call": {
        "id": "0_1001_1515575009",
        "freq": "419000000",
        "sysNum": "0",
        "shortName": "SYS 1",
        "talkgroup": "1001",
        "talkgrouptag": "TG 77",
        "elasped": "0",
        "length": "0",
        "state": "1",
        "phase2": "false",
        "conventional": "false",
        "encrypted": "false",
        "emergency": "false",
        "startTime": "1515575009",
        "stopTime": "1515575009",
        "freqList": [
            {
                "freq": "419000000",
                "time": "1515575009"
            }
        ],
        "recNum": "0",
        "srcNum": "0",
        "recState": "3",
        "analog": "false",
        "filename": "\/home\/*****\/captures\/SYS 1\/2018\/1\/10\/1001-1515575009_4.19e+08.wav",
        "statusfilename": "\/home\/*****\/captures\/SYS 1\/2018\/1\/10\/1001-1515575009_4.19e+08.json"
    },
    "type": "call_start",
    "instanceId": "",
    "instanceKey": ""
}
```

## call_end
```json
{
    "call": {
        "id": "0_1001_1515575009",
        "freq": "419000000",
        "sysNum": "0",
        "shortName": "SYS 1",
        "talkgroup": "1001",
        "talkgrouptag": "TG 77",
        "elasped": "9",
        "length": "3.4199999999999999",
        "state": "2",
        "phase2": "false",
        "conventional": "false",
        "encrypted": "false",
        "emergency": "false",
        "startTime": "1515575009",
        "stopTime": "1515575018",
        "freqList": [
            {
                "freq": "419000000",
                "time": "1515575009"
            }
        ],
        "recNum": "0",
        "srcNum": "0",
        "recState": "2",
        "analog": "false",
        "filename": "\/home\/*****\/captures\/SYS 1\/2018\/1\/10\/1001-1515575009_4.19e+08.wav",
        "statusfilename": "\/home\/*****\/captures\/SYS 1\/2018\/1\/10\/1001-1515575009_4.19e+08.json"
    },
    "type": "call_end",
    "instanceId": "",
    "instanceKey": ""
}
```


## recorders
```json
{
    "recorders": [
        {
            "id": "0_0",
            "type": "P25",
            "srcNum": "0",
            "recNum": "0",
            "count": "6",
            "duration": "76.859999999999999",
            "state": "2",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        },
        {
            "id": "0_1",
            "type": "P25",
            "srcNum": "0",
            "recNum": "1",
            "count": "1",
            "duration": "10.44",
            "state": "2",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        },
        {
            "id": "0_2",
            "type": "P25",
            "srcNum": "0",
            "recNum": "2",
            "count": "0",
            "duration": "1.5147569426240483e-314",
            "state": "2",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        },
        {
            "id": "0_3",
            "type": "P25",
            "srcNum": "0",
            "recNum": "3",
            "count": "0",
            "duration": "1.515774288511435e-314",
            "state": "2",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        },
        {
            "id": "0_4",
            "type": "P25",
            "srcNum": "0",
            "recNum": "4",
            "count": "0",
            "duration": "1.5147569426240483e-314",
            "state": "2",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        },
        {
            "id": "0_5",
            "type": "P25",
            "srcNum": "0",
            "recNum": "5",
            "count": "0",
            "duration": "1.5147569426240483e-314",
            "state": "2",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        },
        {
            "id": "0_6",
            "type": "P25",
            "srcNum": "0",
            "recNum": "6",
            "count": "0",
            "duration": "1.5147569426240483e-314",
            "state": "2",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        },
        {
            "id": "0_7",
            "type": "P25",
            "srcNum": "0",
            "recNum": "7",
            "count": "0",
            "duration": "1.5147569426240483e-314",
            "state": "2",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        },
        {
            "id": "0_8",
            "type": "P25C",
            "srcNum": "0",
            "recNum": "8",
            "count": "0",
            "duration": "76.319999999999993",
            "state": "3",
            "status_len": "0",
            "status_error": "0",
            "status_spike": "0"
        }
    ],
    "type": "recorders",
    "instanceId": "",
    "instanceKey": ""
}
```

## recorder
```json
{
    "recorder": {
        "id": "0_0",
        "type": "P25",
        "srcNum": "0",
        "recNum": "0",
        "count": "5",
        "duration": "48.600000000000001",
        "state": "3",
        "status_len": "0",
        "status_error": "0",
        "status_spike": "0"
    },
    "type": "recorder",
    "instanceId": "",
    "instanceKey": ""
}
```