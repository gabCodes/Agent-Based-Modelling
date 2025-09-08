#from past.builtins import execfile

for i in range(10):
  print("#########################\n"
        "#########################\n"
        "#########################\n"
        "########        #########\n"
        "########  NEXT  #########\n"
        "########        #########\n"
        "#########################\n"
        "#########################\n"
        "#########################")
  
with open("run_me.py") as f:
      code = f.read()
exec(code, globals())
  
  #execfile("run_me.py")