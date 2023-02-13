# AsyncDNSServer: an asynchronous DNS server for the ESP

Built on AsyncUDP, inspired by the standard DNSServer implementation.

Usage is almost a drop-in replacement for DNSServer. The only usage difference is not calling handleNextRequest() in the loop (no need for it).

20230214 Modified to fix ESPAsyncUDP.h not found error.