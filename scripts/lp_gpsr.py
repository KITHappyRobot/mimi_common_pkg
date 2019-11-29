# !/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import re
import copy
import datetime

import xml.etree.ElementTree as ET
import Levenshtein as lev
import fuzzy
#import nltk
import subprocess


class GPSR_data(object):
    def __init__(self):
        self._GetNamesData_()

        # [start]Category1.txtに保存されたデータを配列に入れる
        f = open("/home/rei/catkin_ws/src/gpsr/resource/Category1.txt")
        datas = f.read()
        f.close()
        self.data_sent_list_nb = datas.split('\n')
        # [end]

        self.stack = []
        self.vbgo, self.vbtake, self.vbfind, self.vbspeak, self.vbdeliver, self.vbplace\
        = "go", "grasp", "search", "speak", "give", "place"

    # xmlからオブジェクトとか部屋の名前を取得する
    # イニシャライザの中で呼び出される
    def _GetNamesData_(self):
        path = os.path.abspath("/home/rei/catkin_ws/src/gpsr/resource/GPSRCmdGen-2018-Montreal/CommonFiles")
        self.Obj_root = ET.parse(os.path.join(path, "Objects.xml")).getroot()
        Nam_root = ET.parse(os.path.join(path, "Names.xml")).getroot()
        self.Loc_root = ET.parse(os.path.join(path, "Locations.xml")).getroot()
        Ges_root = ET.parse(os.path.join(path, "Gestures.xml")).getroot()

        self.CategoryData_word_list_nb = []
        self.ObjectData_word_list_nb = []
        self.NameData_word_list_nb = []
        self.RoomData_word_list_nb = []
        self.LocationData_word_list_nb = []
        #self.BeaconData_word_list_nb = []
        #self.PlacementData_word_list_nb = []
        #self.GestureData_word_list_nb = []

        for ctg in self.Obj_root:
            self.CategoryData_word_list_nb.append(ctg.get("name"))
            for obj in ctg:
                self.ObjectData_word_list_nb.append(obj.get("name"))

        for nam in Nam_root:
            self.NameData_word_list_nb.append(nam.text)

        for rom in self.Loc_root:
            self.RoomData_word_list_nb.append(rom.get("name"))
            for loc in rom:
                self.LocationData_word_list_nb.append(loc.get("name"))
                #if loc.get("isBeacon"):
                    #self.BeaconData_word_list_nb.append(loc.get("name"))
                #if loc.get("isPlacement"):
                    #self.PlacementData_word_list_nb.append(loc.get("name"))

        #for ges in Ges_root:
            #self.GestureData_word_list_nb.append(ges.get("name"))


    # オブジェクトとか部屋の名前をまとめる
    # ex) apple -> {object}, bed -> {location}
    def ChangeNames(self, sentence):
        for i in self.CategoryData_word_list_nb:
            sentence = sentence.replace(i, "{category}")
        for i in self.ObjectData_word_list_nb:
            sentence = sentence.replace(i, "{object}")
        for i in self.NameData_word_list_nb:
            sentence = sentence.replace(i, "{name}")
        for i in self.RoomData_word_list_nb:
            sentence = sentence.replace(i, "{room}")
        for i in self.LocationData_word_list_nb:
            sentence = sentence.replace(i, "{location}")
        # for in in self.GestureData_word_list_nb:
        #     sentence = sentence.replace(i, "{gesture}")

        return sentence


    # {~}からオブジェクトとか部屋の名前に戻す
    def Comparsion(self, list, list_names):
        dmeta = fuzzy.DMetaphone()
        cvt_word_list_cnb = copy.deepcopy(self.cvt_word_list_cb)
        try:
            while(isinstance(list[0], int)):
                num = list.pop(0)
                distance = []
                for i in list_names:
                    cvt_word_list_cnb[num] = i
                    list1 = []
                    list2 = []
                    for j in cvt_word_list_cnb:
                        a = dmeta(j)[0]
                        list1.append(a.decode())

                    for k in self.recog_sent_str_nb.split():
                        b = dmeta(k)[0]
                        list2.append(b.decode())

                    #print(' '.join(list1) + "\t\t:" + ' '.join(list2))
                    distance.append(lev.ratio(' '.join(list1), ' '.join(list2))*1.00)
                    #print(str(distance[-1]) + "\t\t:" +' '.join(cvt_word_list_cnb))
                list.append(list_names[distance.index(max(distance))])
                self.result_word_list_nb[num] = list[-1]
                #print(distance)
        except IndexError:
            print("IndexError")
            pass
        except TypeError:
            print("TypeError")
            pass


    def FixSentence(self, sentence):
        self.recog_sent_str_nb = sentence
        cvt_sentence = self.ChangeNames(sentence)

        # 文章全体の編集距離を比較
        distance_max = 0.0
        result_str = "str"
        for i in self.data_sent_list_nb:
            distance = lev.ratio(i, cvt_sentence)*1.00
            if distance_max <= distance and distance >= 0.5:
                distance_max = distance
                result_str = i

        if result_str == "str":
            return "ERROR"

        print(result_str)

        self.cvt_word_list_cb = result_str.lower().replace(',', ' ,').split()
        self.result_word_list_nb = result_str.lower().replace(',', ' ,').split()

        self.category = [i for i, x in enumerate(self.cvt_word_list_cb) if x == "{category}"]
        self.object = [i for i, x in enumerate(self.cvt_word_list_cb) if x == "{object}"]
        self.name = [i for i, x in enumerate(self.cvt_word_list_cb) if x == "{name}"]
        self.room = [i for i, x in enumerate(self.cvt_word_list_cb) if x == "{room}"]
        self.location = [i for i, x in enumerate(self.cvt_word_list_cb) if x == "{location}"]
        self.answer = []

        # {~}からオブジェクトとか部屋の名前に戻す
        # 参照渡しなので注意
        self.Comparsion(self.category, self.CategoryData_word_list_nb)
        self.Comparsion(self.object, self.ObjectData_word_list_nb)
        self.Comparsion(self.name, self.NameData_word_list_nb)
        self.Comparsion(self.room, self.RoomData_word_list_nb)
        self.Comparsion(self.location, self.LocationData_word_list_nb)

        print(' '.join(self.result_word_list_nb))
        return ' '.join(self.result_word_list_nb)

###################################################################

    # 動詞をまとめる
    def Compress(self):
        dt_now = datetime.datetime.now()
        dt_tom = datetime.date.today() + datetime.timedelta(days=1)
        whattosay = {"something about yourself"   : "i am a cute and capable autonomous service robots",
                     "the time"                   : "it is " + dt_now.strftime("%H %M"),
                     "what day is today"          : "it is " + dt_now.strftime("%B %d"),
                     "what day is tomorrow"       : "it is " + dt_tom.strftime("%B %d"),
                     "your team's name"           : "our team's name is KIT happy robot",
                     "your team's country"        : "our team's country is Japan",
                     "your team's affiliation"    : "our team's affiliation is kanazawa institution of technology",
                     "the day of the week"        : "it is " + dt_now.strftime("%A") + " today",
                     "the day of the month"       : "it is " + dt_now.strftime("%B") + " now",
                     "a joke"                     : "you are cool",
                     "name of person"             : "Alex",
                     "how many {object} there are on the {location}" : "one"}

        cmp_sent_str_cb = ' '.join(self.cvt_word_list_cb)

        # go <- go, navigate, enter
        # cmp_sent_str_cb = [self.vbgo if x in {"go", "navigate", "enter"} else x for x in cmp_sent_str_cb]
        cmp_sent_str_cb = cmp_sent_str_cb.replace("go", self.vbgo).replace("navigate", self.vbgo).replace("enter", self.vbgo)
        # grasp <- get, grasp, take, pick up
        cmp_sent_str_cb = cmp_sent_str_cb.replace("get", self.vbtake).replace("grasp", self.vbtake).replace("take", self.vbtake).replace("pick up", self.vbtake)
        # search <- find, locate, look for
        cmp_sent_str_cb = cmp_sent_str_cb.replace("find", self.vbfind).replace("locate", self.vbfind).replace("look for", self.vbfind)
        # speak <- tell, say
        cmp_sent_str_cb = cmp_sent_str_cb.replace("tell", self.vbspeak).replace("say", self.vbspeak)
        # place <- put, place
        cmp_sent_str_cb = cmp_sent_str_cb.replace("put", self.vbplace).replace("place", self.vbplace)
        # pass <- bring, deliver, give
        cmp_sent_str_cb = cmp_sent_str_cb.replace("bring", self.vbdeliver).replace("deliver", self.vbdeliver).replace("give", self.vbdeliver)

        for i, w in whattosay.items():
            if i in cmp_sent_str_cb:
                print("okin")
                cmp_sent_str_cb = cmp_sent_str_cb.replace(i, "{answer}")
                self.answer.append(w)
        print("okout")

        cmp_sent_str_cb = cmp_sent_str_cb.replace(" the ", " ")

        if "it" in cmp_sent_str_cb:
            cmp_sent_str_cb = cmp_sent_str_cb.replace("it", "{object}")
            self.object.append(self.object[-1])

        # if "me" in cmp_sent_str_cb:
        #     cmp_sent_str_cb = cmp_sent_str_cb.replace(" to me", " to {name} at {location}")
        #     cmp_sent_str_cb = cmp_sent_str_cb.replace(" me", " to {name} at {location}")
        #     if cmp_sent_str_cb.find("me")/len(cmp_sent_str_cb) < 0.5:
        #         self.name.insert(0, "operator")
        #         self.location.insert(0, "operator")
        #     else:
        #         self.name.append("operator")
        #         self.location.append("operator")

        # if "name of person" in cmp_sent_str_cb:
        #     # if "{room}" in cmp_sent_str_cb:
        #     #     cmp_sent_str_cb = self.vbgo + " to {room} , speak {answer} to {name} at {location}"
        #     # else:
        #     #     cmp_sent_str_cb = self.vbgo + " to {location} , speak {answer} to {name} at {location}"
        #     #     self.location[0], self.location[1] = self.location[1], self.location[0]
        #     self.answer.append(self.NameData_word_list_nb[0]) # 人の名前を取得
        #
        # if "how many {object} there are on {location}" in cmp_sent_str_cb:
        #     #cmp_sent_str_cb = cmp_sent_str_cb.replace("how many {object} there are on {location}", " , speak {answer}")
        #     self.answer.append("one") # オブジェクトの数を取得
        #     #self.location.pop(-1)

        # if self.vbdeliver + " {object} to {name} at {location}" in cmp_sent_str_cb:
        #     cmp_sent_str_cb = self.vbdeliver + " to {name} "
        #
        # if self.vbplace + " {object} on {location}" in cmp_sent_str_cb:
        #     cmp_sent_str_cb = cmp_sent_str_cb.replace(self.vbplace + " {object} on {location}", self.vbtake + " {object} from {location} , " + self.vbgo + " {location} , " + self.vbplace + " {location}")

        #print("圧縮後:" + cmp_sent_str_cb)
        return cmp_sent_str_cb

    def GetObjectName(self, categoryname_str):
        if categoryname_str == "none":
            return "none"
        for ctg in self.Obj_root:
            for obj in ctg:
                if ctg.get("name") == categoryname_str and obj.get("difficulty") == "easy":
                    return obj.get("name")

    def GetLocationName(self, objectname_str):
        if objectname_str == "none":
            return "none"
        for ctg in self.Obj_root:
            for obj in ctg:
                if obj.get("name") == objectname_str:
                    return ctg.get("defaultLocation")

    def GetRoomName(self, locationname_str):
        if locationname_str == "none":
            return "none"
        for rom in self.Loc_root:
            for loc in rom:
                if loc.get("name") == locationname_str:
                    return rom.get("name")

    def MakeAction(self):
        cmp_sent_str_cb = self.Compress()
        for i in range(3):
            self.category.append("none")
            self.object.append("none")
            self.name.append("none")
            self.room.append("none")
            self.location.append("none")
            self.answer.append("none")

        action = {
            self.vbspeak + " {answer} to {name} at {location}"                                                                          : [[self.vbgo, self.GetRoomName(self.location[0]), "none", "none"], [self.vbfind, "none", self.name[0], "none"], [self.vbspeak, "none", "none", self.answer[0]]],
            self.vbspeak + " me name of person in {room}"                                                                               : [[self.vbgo, self.room[0], "none", "none"], [self.vbfind, "none", "person", "none"], [self.vbgo, "operator", "none", "none"], [self.vbspeak, "none", "none", "Alex"]],
            self.vbspeak + " me name of person at {location}"                                                                           : [[self.vbgo, self.location[0], "none", "none"], [self.vbfind, "none", "person", "none"], [self.vbgo, self.location[0], "none", "none"], [self.vbfind, "none", "person", "none"], [self.vbgo, "operator", "none", "none"], [self.vbspeak, "none", "none", "Alex"]],
            self.vbspeak + " me how many {object} there are on {location}"                                                              : [[self.vbgo, self.location[0], "none", "none"], [self.vbfind, "none", "object", "none"], [self.vbgo, "operator", "none", "none"], [self.vbspeak, "none", "none", "one"]],
            self.vbtake + " {object} from {location} and " + self.vbplace + " {object} on {location}"                                   : [[self.vbgo, self.location[0], "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, self.location[1], "none", "none"], [self.vbplace, "none", self.object[0], "none"]],
            self.vbtake + " {object} and " + self.vbplace + " {object} on {location}"                                                   : [[self.vbgo, self.GetLocationName(self.object[0]), "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, self.location[0], "none", "none"], [self.vbplace, "none", self.object[0], "none"]],
            self.vbtake + " {object} from {location} and " + self.vbdeliver + " {object} to {name} at {location}"                       : [[self.vbgo, self.location[0], "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, self.GetRoomName(self.location[0]), "none", "none"], [self.vbfind, "none", self.name[0], "none"], [self.vbdeliver, "none", self.object[0], "none"]],
            self.vbtake + " {object} from {location} and " + self.vbdeliver + " {object} to me"                                         : [[self.vbgo, self.location[0], "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, "operator", "none", "none"], [self.vbdeliver, "none", self.object[0], "none"]],
            self.vbplace + " {object} on {location}"                                                                                    : [[self.vbgo, self.GetLocationName(self.object[0]), "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, self.location[0], "none", "none"], [self.vbplace, "none", self.object[0], "none"]],
            self.vbgo + " to {room}, " + self.vbfind + " {name}, and " + self.vbspeak + " {answer}"                                     : [[self.vbgo, self.room[0], "none", "none"], [self.vbfind, "none", self.name[0], "none"], [self.vbspeak, "none", "none", self.answer[0]]],
            #self.vbgo + " to {room}, " + self.vbfind + " {name}, and answer a question"                                                 : ,
            self.vbgo + " to {location}, " + self.vbfind + " {object}, and " + self.vbdeliver + " {object} to {name} at {location}"     : [[self.vbgo, self.location[0], "none", "none"], [self.vbfind, "none", self.object[0], "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, self.GetRoomName(self.location[0]), "none", "none"], [self.vbfind, "none", self.name[0], "none"], [self.vbdeliver, "none", self.object[0], "none"]],
            self.vbgo + " to {location}, " + self.vbfind + " {object}, and " + self.vbdeliver + " {object} to me"                       : [[self.vbgo, self.location[0], "none", "none"], [self.vbfind, "none", self.object[0], "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, "operator", "none", "none"], [self.vbdeliver, "none", self.object[0], "none"]],
            self.vbgo + " to {location}, " + self.vbfind + " {object}, and " + self.vbplace + " {object} on {location}"                 : [[self.vbgo, self.location[0], "none", "none"], [self.vbfind, "none", self.object[0], "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, self.location[1], "none", "none"], [self.vbplace, "none", self.object[0], "none"]],
            self.vbfind + " {object} in {room}"                                                                                         : [[self.vbgo, self.GetLocationName(self.object[0]), "none", "none"], [self.vbfind, "none", self.object[0], "none"]],
            self.vbfind + " {category} in {room}"                                                                                       : [[self.vbgo, self.GetLocationName(self.GetObjectName(self.category[0])), "none", "none"], [self.vbfind, "none", self.GetObjectName(self.category[0]), "none"]],
            self.vbfind + " {name} in {room} and " + self.vbspeak + " {answer}"                                                         : [[self.vbgo, self.room[0], "none", "none"], [self.vbfind, "none", self.name[0], "none"], [self.vbspeak, "none", "none", self.answer[0]]],
            #self.vbfind + " {name} in {room} and answer a question"                                                                     : ,
            self.vbdeliver + " me {object} from {location}"                                                                             : [[self.vbgo, self.location[1], "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, "operator", "none", "none"], [self.vbdeliver, "none", self.object[0], "none"]],
            self.vbdeliver + " {object} to {name} at {location}"                                                                        : [[self.vbgo, self.GetLocationName(self.object[0]), "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, self.GetRoomName(self.location[0]), "none", "none"], [self.vbfind, "none", self.name[0], "none"], [self.vbdeliver, "none", self.object[0], "none"]],
            self.vbdeliver + " {object} to me"                                                                                          : [[self.vbgo, self.GetLocationName(self.object[0]), "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, "operator", "none", "none"], [self.vbdeliver, "none", self.object[0], "none"]],
            self.vbdeliver + " me {object}"                                                                                             : [[self.vbgo, self.GetLocationName(self.object[0]), "none", "none"], [self.vbtake, "none", self.object[0], "none"], [self.vbgo, "operator", "none", "none"], [self.vbdeliver, "none", self.object[0], "none"]],
            #"answer a question to {name} at {location}"                                                                                 :,
        }


        for i, x in action.items():
            if i == cmp_sent_str_cb:
                self.stack = x
                self.stack.append(["end", "none", "none", "none"])

        return self.stack

    # 分節ごとに分割する
    # def SentenceToClause(self):
    #     cmp_word_list_cb = self.Compress().split()
    #     clause_word_list_cb = []
    #
    #     # "and"と","の要素番号を取得
    #     indexes = [i for i, x in enumerate(cmp_word_list_cb) if x in {"and", ","}]
    #     indexes.append(len(cmp_word_list_cb))
    #
    #     # 文節に分けてリストに保存
    #     start = 0
    #     for i, w in enumerate(indexes):
    #         clause_word_list_cb.append(cmp_word_list_cb[start:w])
    #         start = w+1
    #     clause_word_list_cb = [x for x in clause_word_list_cb if x]
    #     print(clause_word_list_cb)
    #     return clause_word_list_cb
    #
    #
    # def ClauseToPhrase(self):
    #     clause_word_list_cb = self.SentenceToClause()
    #     phrase_word_list_cb = []
    #     clauselist = []
    #
    #     # to, at, from, on, inの要素番号を取得
    #     for i, w in enumerate(clause_word_list_cb):
    #         index = [j for j, x in enumerate(w) if x in {"to", "at", "from", "on", "in"}]
    #
    #         phraselist = [clause_word_list_cb[i][0]]
    #         for k, n in enumerate(index):
    #             if n != 1 and k == 0:
    #                 phraselist.append(w[1:n])
    #             phraselist.append(w[n:n+2])
    #
    #         if len(index) == 0:
    #             phraselist.append(w[1:2])
    #         elif index[-1] != len(w)-2:
    #             phraselist.append(w[len(w)-1:])
    #
    #         phrase_word_list_cb.append(phraselist)
    #     print(phrase_word_list_cb)
    #     return phrase_word_list_cb
    #
    #
    # def MakeAction(self):
    #     phrase_word_list_cb = self.ClauseToPhrase()
    #
    #     # 動詞ごとにaction文生成へ
    #     # self.vbgo, self.vbtake, self.vbfind, self.vbspeak, self.vbdeliver, self.vbplace\
    #     # = "go", "grasp", "search", "speak", "give", "place"
    #     for i in phrase_word_list_cb:
    #         for j in i:
    #             if not isinstance(j, str):
    #                 for k in j:
    #                     if k in {"to", "at", "from", "on", "in"}:
    #                         #if i[0] != self.vbplace and i[0] != self.vbdeliver:
    #                             #self.stack.append()
    #                         for l, w in action.items():
    #                             if (i[0] + k) == l:
    #                                 self.stack.append([w, ])
    #
    #                         print(i[0] + k + ":" + j[1])
    #                         break
    #                     else:
    #                         print(i[0] + ":" + j[0])
    #
    #         # if i[0] == self.vbgo:
    #         #     self.stack.append([self.vbgo, location, "none", "none"])
    #         # elif i[0] == self.vbtake:
    #         #     self.stack.append([self.vbtake, "none", object, "none"])
    #         # elif i[0] == self.vbfind:
    #         #     self.stack.append([self.vbfind, "none", object, "none"])
    #         # elif i[0] == self.vbspeak:
    #         #     self.stack.append([self.vbspeak, "none", "none", answer])
    #         # elif i[0] == self.vbdeliver:
    #         #     self.stack.append([self.vbdeliver, "none", name, "none"])
    #         # elif i[0] == self.vbplace:
    #         #     self.stack.append([self.vbplace, location, "none", "none"])
    #         # else:
    #         #     pass

if __name__ == '__main__':
    lp = GPSR_data()
    string = raw_input()
    print("answer :" + lp.FixSentence(string))
    print(lp.MakeAction())
