# Create A New Client Module
In DeepClaw Server-Client system, we use socket to establish the communication between server and client. In this document, we propose the process to create a new module in client end.

We introduce the module - `HelloDC.py` as the example.

Firstly, you should add your algorithm module into server end, more details can be found [here](https://github.com/ancorasir/DeepClaw/blob/master/docs/others/Add%20New%20Module%20in%20Server.md).

`HelloDC.py` in server:

```python
# Copyright (c) 2020 by BionicLab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File:
@Author:
@Date:
@Description: 
"""
# import packages here


class HelloDC(object):
    def __init__(self, name: str):
        self.name = name

    def run(self, user_name: str):
        response = 'Hello, '+user_name+'! My name is '+self.name+'.'
        return response

# static functions here
```

`HelloDC-client.py` should be:

```python
# Copyright (c) 2020 by BionicLab. All Rights Reserved.
# -*- coding:utf-8 -*-
"""
@File:
@Author:
@Date:
@Description:
"""
import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(ROOT)

from utils.Client import Client


class HelloDC(object):
    def __init__(self, name: str):
        data = {'type': 'instruction',
                'data': ['from modules.HelloDC import HelloDC',
                         [(name)],
                         'HelloDC']}
        self.client = Client(host_ip='0.0.0.0', port=2020)
        self.client.start()
        self.client.send(data)
        self.client.close()

    def run(self, user_name: str):
        self.client.start()
        data = {'type': 'string',
                'data': user_name}
        self.client.send(data)
        feedback = self.client.recv(MSG_WAITALL)
        return feedback['feedback']

# static functions here
```

In `HelloDC-client.py`, we create two types of message: "instruction" message to instantiate the module in server, and others like "string" or "image" to transmit parameters to `run()` function in server. So, to create your own client module, you only need to modify the information of data.

For example, if your module have name "MyModule", and require two parameters "arg1" and "arg2" to be initialed. The data in `__init__(self, arg1, arg2)` should be

```python
data = {'type': 'instruction',
                'data': ['from modules.MyModule import MyModule',
                         [(arg1, arg2)],
                         'MyModule']}
```

And the data in `run(self, other_arg1, other_arg2)` should be

```python
data = {'type': 'tuple',
                'data': (ohter_arg1, other_arg2)}
```

