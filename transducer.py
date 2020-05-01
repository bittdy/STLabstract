class Transducer(object):
    def __init__(self, interval, parent, child, bound):
        self.interval = interval
        self.parent = parent
        self.child = child
        self.bound = bound

class Once(Transducer):
    def __init__(self, interval, parent, child, bound):
        super(Once, self).__init__(interval, parent, child, bound)

    def trans(self):
        if self.parent.t < (self.interval - self.bound[1]) and self.child.t < (self.interval - self.bound[1]):  #this subformula is not actived
            return None
        elif self.parent.t < (self.interval - self.bound[1]) and \
                self.child.t > (self.interval - self.bound[1]) and self.child.t < (self.interval - self.bound[0]): #just actived
            if any(self.child.state_path['Once'][self.bound]):           #the construct of the state path need to find the path in the bound
                return True
            else:
                return None
        elif self.parent.t < (self.interval - self.bound[1]) and self.child.t > (self.interval - self.bound[0]): #run through
            if any(self.child.state_path['Once'][self.bound]):
                return True
            else:
                return False
        elif self.parent.t > (self.interval - self.bound[1]) and self.parent.t < (self.interval - self.bound[0]) and \
                self.child.t > (self.interval - self.bound[1]) and self.child.t < (self.interval - self.bound[0]):     #all active
            if self.parent.state['Once'][self.bound] == True:
                return True
            if any(self.child.state_path['Once'][self.bound]):
                return True
            else:
                return None

        elif self.parent.t > (self.interval - self.bound[1]) and self.parent.t < (self.interval - self.bound[0]) and \
                self.child.t > (self.interval - self.bound[0]):
            if self.parent.state['Once'][self.bound] == True:
                return True
            if any(self.child.state_path['Once'][self.bound]):
                return True
            else:
                return False
        elif self.parent.t > (self.interval - self.bound[0]) and self.child.t > (self.interval - self.bound[0]):
            if self.parent.state['Once'][self.bound] == True:
                return True
            else:
                return False


# class Always(Transducer):
#     def __init__(self, init_state, init_output, bound):
#         super(Always, self).__init__(init_state, init_output, bound)
#
#     def trans(self, current_state, read_state):
#         new_state = current_state
#         output = None
#         if current_state == '' and read_state == '':
#             new_state =
#             output =
#         elif
#
#         return new_stete, output
