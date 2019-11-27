# 概要
mimi_common_pkgは、mimiの制御において使用頻度の高いものを纏めたROSパッケージです。以下、パッケージの構成要素を記します。

## scripts

scriptsには、使用頻度の高いアクションクライアントを纏めたファイル`common_action_client`や、使用頻度の高い関数を纏めた`common_function`などが存在します。

## action_server

action_serverには、使用頻度の高いアクションサーバーファイルが収納されています。

## action
actionには、action_server内で使用されるアクションファイルが収納されています。

## config

configには、本パッケージで生成されるパラメータを保存するyamlファイルが収納されています。

## launch

launchには、アクションサーバーを一斉に起動するためのlaunchファイルなどが収納されています。
