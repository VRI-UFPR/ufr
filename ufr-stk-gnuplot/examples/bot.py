import ufr


server = ufr.Server("@new telegram @token 8156512523:AAHdvmUYzxnTAOb3IvsItDjcqdRoDeVXHSw")

msg = server.get("^s")
print(msg)
server.put("Opa\n\n")

server.close()