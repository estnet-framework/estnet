ESTNeT has a publisher subscriber model.
This can be used to independently publish values and subscribe to these values if needed.
## Publisher
To create a publisher, your module needs to inherit from the Publisher interface. This needs to be done in the NED and the class implementing your module.
### NED
```
import estnet.siminterface.pubsub.Publisher;

simple YourModule like Publisher {
... 
}
```
### C++
```
#include "estnet/siminterface/pubsub/SimplePublisher.h"

class YourModule : public estnet::SimplePublisher {
  ...
}
```
Remember to call `SimplePublisher::initialize()` in the initialize of your module. In order to publish a value, use  `publishValue(<yourValue>, <key>)`.
## Subscriber
To create a subscriber, your C++ class needs to inherit the Subscriber interface and implement its method `void receivedPubSubMessage(estnet::PubSubMsg *pubSubMsg)`, for example:
```
#include "estnet/siminterface/pubsub/Subscriber.h"

class PowerManager : public cSimpleModule, public estnet::Subscriber {
  virtual void receivedPubSubMessage(estnet::PubSubMsg *pubSubMsg);
  ...
}
```
In the initialize of your module, you can then call `subscribeTopic(string key)` to receive message published to this key. Multiple topics can be subscribed.
Every message from the subscribed topics will then call `receivedPubSubMessage`, where a PubSubMsg contains a key and a value. These can be used for example like this:
```
void PowerManager::receivedPubSubMessage(estnet::PubSubMsg *pubSubMsg) {
    if((std::string)pubSubMsg->key == "/omnet/power/consumption"){
      _power -= atoi((pubSubMsg->value).c_str());
    }
    if((std::string)pubSubMsg->key == "/omnet/power/production"){
      _power += atoi((pubSubMsg->value).c_str());
    }

    EV << "New Powerlevel is " << _power << std::endl;
}
```
## Keys
The keys, to which values get published are compatible with MQTT topics. These should follow a filepath like structure, for example "sat1/power/solarpanel1/status".
### Wildcards
The power of the structured approach comes into play, when combined with wildcards. There are single- and multi-level wildcards. Note that only subscriptions can contain wildcards, publishing with a wildcard would lead to ambiguity.
#### Single level
The single-level wildcard is a "+". This will match with any value for exactly one level, for example subscribing to "sat1/power/+/status" will get messages both on the "sat1/power/solarpanel1/status" and the "sat1/power/battery/status" key.
#### Multi level
The multi-level wildcard is a "#". This will match the whole following tree structure, so it can only be placed at the end of an key. "sat1/power/#" will get all messages to keys starting with "sat1/power", for example "sat1/power/battery/charge" and "sat1/power/status".