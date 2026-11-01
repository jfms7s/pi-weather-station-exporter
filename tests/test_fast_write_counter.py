import threading

import Weather_Station as ws


def test_fresh_counter_reads_zero_every_time():
    counter = ws.FastWriteCounter()
    assert [counter.value() for _ in range(5)] == [0] * 5


def test_reads_do_not_count_as_increments():
    counter = ws.FastWriteCounter()
    for _ in range(7):
        counter.increment()
    assert [counter.value() for _ in range(3)] == [7, 7, 7]


def test_concurrent_increments_and_reads_lose_nothing():
    writers, increments = 4, 50_000
    counter = ws.FastWriteCounter()
    done = threading.Event()

    def write():
        for _ in range(increments):
            counter.increment()

    def read():
        while not done.is_set():
            counter.value()

    readers = [threading.Thread(target=read) for _ in range(2)]
    writer_threads = [threading.Thread(target=write) for _ in range(writers)]
    for thread in readers + writer_threads:
        thread.start()
    for thread in writer_threads:
        thread.join()
    done.set()
    for thread in readers:
        thread.join()

    assert counter.value() == writers * increments
