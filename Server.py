import asyncore,socket, os

class clientHandler(asyncore.dispatcher):
    
    def __init__(self, socket, address, chunk_size = 64):
        asyncore.dispatcher.__init__(self,sock = socket)
        self.chunk_size = chunk_size
        self.address = address
        self.socket = socket
        self.file = open('standing.txt', 'w')
        
    def handle_read(self):
        data = self.recv(self.chunk_size)
        self.file.write(data)
        print data

    def handle_close(self):
        print "Closing client " + str(address)
        self.file.close()
        self.close()
        
class Server(asyncore.dispatcher):
    def __init__(self, address):
        asyncore.dispatcher.__init__(self)
        self.create_socket(socket.AF_INET,socket.SOCK_STREAM)
        self.bind(address)
        self.listen(5)
        
    def handle_accept(self):
        client, addr = self.accept()
        print "Connected to " + str(addr)
        clientHandler(socket = client,address = addr)
        
    def handle_close(self):
        self.close()
        print "Closing server."

if __name__ == '__main__':
    address = ('0.0.0.0', 5000)
    server = Server(address)
    try:
        print "Server listening on " + str(address) 
        asyncore.loop(0.2,use_poll = True)
    except KeyboardInterrupt:
        print "Closing server."
        server.close()
        
        