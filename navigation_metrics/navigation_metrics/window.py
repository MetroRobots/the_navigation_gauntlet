class TimeWindow:
    def __init__(self, start, end):
        self.start = start
        self.end = end

    def __contains__(self, bmsg):
        return self.start <= bmsg.t and bmsg.t <= self.end

    def in_range(self, seq):
        new_seq = []
        for bmsg in seq:
            if bmsg in self:
                new_seq.append(bmsg)
        return new_seq

    def length(self):
        return self.end - self.start


class WindowBag:
    def __init__(self, bag, window):
        self.bag = bag
        self.window = window

    def __contains__(self, topic):
        return topic in self.bag

    def get_topics_by_type(self, type_):
        return self.bag.get_topics_by_type(type_)

    def get_parameter(self, name, default_value):
        return self.bag.get_parameter(name, default_value)

    def __getitem__(self, arg):
        return self.window.in_range(self.bag[arg])

    def length(self):
        return self.window.length()

    def __repr__(self):
        return f'{self.window.start - self.bag.get_start_time()} - {self.window.end - self.bag.get_start_time()}'
