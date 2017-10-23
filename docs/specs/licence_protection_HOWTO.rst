
Building a licence protection system
====================================

From
----

https://crypto.stackexchange.com/questions/16483/creating-a-license-system-based-on-asymmetric-encryption-rsa-or-ecdsa


Best answer
-----------

Sign the license text. There's nothing confidential in the license text, 
so you don't need confidentiality.

If possible, include some kind of customer ID or something in the license 
that links it to a particular customer.

Standard digital signature schemes can easily sign arbitrary-length messages. 
Don't worry about the length of the message. (Internally, they hash the 
message with a collision-resistant cryptographic hash, then sign the hash, 
roughly speaking, which is how they can handle arbitrary-length messages 
efficiently. But you shouldn't need to know that, if you are using a 
standard implementation of a well-vetted crypto algorithm.)

One approach to revocation is to have your client software contact a 
central server for periodic download of the revocation list. The 
alternative is to have your server issue short-lived signed authorizations, 
and require the client to periodically contact the server when each one 
expires to get a new one (and the server can withhold providing any new 
authorizations once you revoke that customer). In either case, make sure 
that the client can continue to use your software for some set period while 
they are offline without destroying their access to the software, or your 
customers might become very mad at you.

There is no purely offline solution to revocation. If the customer has 
no network connectivity, there's no way to revoke their software license 
early. If in the absence of revocation their software license would be 
valid until time T, and at time t<T you want to revoke them, you can't. 
Consider: from the client's perspective, there is nothing that lets them 
distinguish between whether they are revoked or not, if they don't have 
any communication whatsoever with the server or anyone else. So if you 
expect clients to be offline, it becomes a tradeoff between "the lifetime 
of a license" (how long before it expires?) vs "speed of revocation" (how 
long before revocation takes effect?). If you issue licenses that are 
good for one year, then revocation won't take effect until that year is 
up -- if clients are offline.

What you can do is refuse to provide software upgrades to clients whose 
software has been upgraded. This won't prevent them from continuing to 
use the older version of the software, though, for as long as their 
license is valid. Optionally, you could also have the client software 
ry to connect to the Internet whenever possible, and if the client goes 
for a certain time period without having being able to reach the server, 
you could have the client disable itself; whether this is acceptable will 
depend upon how you expect your legitimate customers will use your software.

If your private signing key is stolen, you are hosed. There's nothing you 
can do to prevent whoever holds that signing key from generating illegitimate 
licenses that the existing version of your software will accept as valid. 
All you can do is change the public key that's hardcoded into the software, 
in the next software update (so people won't be able to use that newer v
ersion of software using licenses signed by the old stolen key) -- but 
this won't stop people from using the older version of your software. So 
try to not let your signing key get stolen.

Finally, remember that these licensing schemes provide a speedbump, not 
strong security. They exist to keep honest people honest, but they will 
not be effective at deterring or stopping dedicated malicious people from 
bypassing your copy protection system. Therefore, whatever you do, try to 
ensure that your license system doesn't inconvenience honest users too 
much. Don't drive them to download pirated copies because they're less 
annoying than the actual paid-up copy.
