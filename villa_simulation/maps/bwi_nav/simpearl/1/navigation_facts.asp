#program base.

room(hallway).
room(lab).
room(couch).

acc(hallway,lab).
acc(lab,couch).

object(poli_1).
object(small_table_1).
object(bookcase_1).
object(big_table_4).
object(big_table_1).
object(couch_1).
object(big_table_3).
object(big_table_2).
object(gemini_1).
object(hallway_2).
object(kitchen_1).
object(hallway_1).

inside(poli_1,lab).
inside(small_table_1,lab).
inside(bookcase_1,lab).
inside(big_table_4,lab).
inside(big_table_1,lab).
inside(couch_1,couch).
inside(big_table_3,lab).
inside(big_table_2,lab).
inside(gemini_1,lab).
inside(hallway_2,hallway).
inside(kitchen_1,lab).
inside(hallway_1,hallway).

% Standard accessibility implication rules
dooracc(R1,D,R2) :- hasdoor(R1,D), hasdoor(R2,D), R1 != R2, door(D), room(R1), room(R2).
dooracc(R1,D,R2) :- dooracc(R2,D,R1).

acc(R1,R1) :- room(R1).
acc(R1,R2) :- acc(R2,R1), room(R1), room(R2).
acc(R1,R2) :- acc(R1,R3), acc(R2,R3), room(R1), room(R2), room(R3).

